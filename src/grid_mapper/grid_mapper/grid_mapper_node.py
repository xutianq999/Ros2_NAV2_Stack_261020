"""把 Super-LIO 的逐帧点云累积成"会生长的"二维占据栅格，发布到 /map。

## 为什么需要这个节点

前沿探索需要一张**会长出未知区域**的占据栅格：前沿（frontier）就是 free 格与
unknown 格的交界，没有 unknown 就没有前沿可探。而本工程默认的 /map 来自
map_server 加载的**静态先验图** —— 它只有最外一圈边界，探索走不出去。

## 为什么不用 slam_toolbox 出这张图

slam_toolbox 也能给实时 /map，但它同时会发布自己的 `map -> odom`（回环修正）。
本工程 Super-LIO 那条链里 `map -> odom` 由 super_lio_bridge 发布（启动位姿，常量），
两者都发同一条 TF 会互相打架。这里选择让 Super-LIO 保持**唯一**的位姿来源：
桥接层照旧发 `map -> odom`，本节点只把点云变成栅格，完全不碰 TF。

代价是没有回环检测，栅格跟着 LIO 漂。实测 Super-LIO 在 104 m 路径上"重定位 vs
建图"轨迹的 RMSE 约 13 mm，单回合建图漂移很小。若将来在长距离场景发现漂移已经
影响前沿质量，再换 slam_toolbox 那条路 —— 届时必须让桥接层停发 `map -> odom`。

## 坐标系

Super-LIO 的点云在它自己的 `world` 帧，而按 super_lio_bridge 的桥接假设，`world`
帧就是 map 帧。所以本节点**直接在点云坐标上累加**，发布时把 OccupancyGrid 的
frame_id 标成 `map_frame`（默认 "map"），不做 TF 变换 —— 那会去找一个并不存在的
`world -> map` 变换。射线起点取 `/lio/odom` 的位置（同一套状态量，同为 world 帧）。

## 占据判定

标准 log-odds 累加：每条射线沿途的格子记 miss（清成 free），端点记 hit（占障）。
- 端点只有落在高度带 `[z_min, z_max]` 内才记 hit。地面在 map 系约 -0.32 m
  （IMU 在 base_footprint 上方约 0.32 m），所以 `z_min = 0.0` 就足以把地面排除，
  否则整块地板会被铺成障碍。落在带外的点仍然参与射线清理（地面点正好用来把地面
  刷成 free）。
- 清理距离上限 `max_clear_range`：更远的射线不再逐个格子清理，只记端点。这既省
  算力，也符合直觉 —— 远处本来就是"看着空但没确认"的区域。
- 每帧抽稀到 `max_points` 个点，控制单帧计算量。
- 一帧的所有格子更新只做一次"去重 + 批量累加"（`np.unique` 拿到出现次数后一次写入），
  而不是逐条射线调用 `np.add.at`：后者每帧要几十万次标量加法，10 Hz 下没有余量。
  实测批量做法单帧 2~8 ms（随每帧点数与栅格规模变化）。
  数值在 int64 里算完、裁剪到 [-24, 32] 再写回 int16，所以不存在溢出问题。

## 关于 livox_360.yaml 里的 blind（实测结论，不是推测）

`lio.sensor.blind` 在这份建图配置里是 2.0 m，乍看像是"车周 2 m 内的点全被丢掉"。
实测不是：订阅 `/lio/cloud_world` 统计 367 帧，全局最近点 **0.41 m**，2 m 内的点
仍占 2.4%。也就是发布出来的点云没有近身空洞，本节点在车身附近拿到的数据是有效的。
（`blind` 影响的是 Super-LIO 内部挑点的范围，不截断这条输出。）
"""

from __future__ import annotations

import array
import time
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField

from grid_mapper.static_hits import StaticHitFilter


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """从 PointCloud2 里取出 (N, 3) 的 float32 坐标。

    自己解析而不用 sensor_msgs_py：那个 API 在 humble/iron 之间签名变过，
    这里只需要 x/y/z，按 offset 直接取更稳，也不吃 Python 逐点循环的开销。
    只支持 float32 字段（本工程的点云都是 PCL 直出的 float32）。
    """
    offsets = {}
    for field in msg.fields:
        if field.name in ('x', 'y', 'z'):
            if field.datatype != PointField.FLOAT32:
                raise ValueError(f'字段 {field.name} 不是 float32，无法解析')
            offsets[field.name] = field.offset
    if len(offsets) != 3:
        raise ValueError(f'点云缺少 x/y/z 字段: {sorted(offsets)}')

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    usable = (raw.size // msg.point_step) * msg.point_step
    raw = raw[:usable].reshape(-1, msg.point_step)

    xyz = np.empty((raw.shape[0], 3), dtype=np.float32)
    for i, name in enumerate(('x', 'y', 'z')):
        off = offsets[name]
        xyz[:, i] = raw[:, off:off + 4].copy().view(np.float32).ravel()

    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]


class GridMapper(Node):
    """逐帧累加 log-odds，按需扩张栅格，周期性发布 OccupancyGrid。"""

    def __init__(self) -> None:
        super().__init__('grid_mapper')

        self.declare_parameter('cloud_topic', '/lio/cloud_world')
        self.declare_parameter('odom_topic', '/lio/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')

        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('z_max', 1.5)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_clear_range', 10.0)
        self.declare_parameter('max_points', 8000)
        self.declare_parameter('ray_stride', 2)
        # Only persistent hits become map obstacles. Raw scans continue to feed
        # both costmaps and collision protection without this delay.
        self.declare_parameter('static_confirmation_time', 2.0)
        self.declare_parameter('static_max_gap', 0.5)
        self.declare_parameter('static_confirmation_resolution', 0.15)

        # log-odds 增量与阈值。整数运算；每次累加在 int64 里算完裁剪后再写回 int16
        self.declare_parameter('hit_increment', 4)
        self.declare_parameter('miss_increment', -1)
        self.declare_parameter('logodds_min', -24)
        self.declare_parameter('logodds_max', 32)
        self.declare_parameter('occupied_threshold', 8)
        self.declare_parameter('free_threshold', -3)

        self.declare_parameter('initial_extent', 20.0)
        self.declare_parameter('publish_rate', 1.0)

        self._res = float(self.get_parameter('resolution').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._z_min = float(self.get_parameter('z_min').value)
        self._z_max = float(self.get_parameter('z_max').value)
        self._min_range = float(self.get_parameter('min_range').value)
        self._max_clear_range = float(self.get_parameter('max_clear_range').value)
        self._max_points = int(self.get_parameter('max_points').value)
        self._ray_stride = max(1, int(self.get_parameter('ray_stride').value))
        self._hit_inc = int(self.get_parameter('hit_increment').value)
        self._miss_inc = int(self.get_parameter('miss_increment').value)
        self._lo_min = int(self.get_parameter('logodds_min').value)
        self._lo_max = int(self.get_parameter('logodds_max').value)
        self._occ_thresh = int(self.get_parameter('occupied_threshold').value)
        self._free_thresh = int(self.get_parameter('free_threshold').value)
        confirmation_resolution = float(
            self.get_parameter('static_confirmation_resolution').value)
        if not np.isfinite(confirmation_resolution) or confirmation_resolution < self._res:
            raise ValueError('Static confirmation resolution must be >= map resolution')
        self._static_hits = StaticHitFilter(
            float(self.get_parameter('static_confirmation_time').value),
            float(self.get_parameter('static_max_gap').value),
            max(1, int(round(confirmation_resolution / self._res))))

        if self._hit_inc * self._max_points > np.iinfo(np.int16).max:
            raise ValueError(
                f'hit_increment({self._hit_inc}) × max_points({self._max_points}) '
                f'会溢出 int16，调小其中一个')


        extent_cells = int(float(self.get_parameter('initial_extent').value) / self._res)
        # map 帧原点就是格子 (0, 0)，开局给一块对称的方形区域，之后按需扩张
        self._min_ix, self._max_ix = -extent_cells, extent_cells
        self._min_iy, self._max_iy = -extent_cells, extent_cells
        self._logodds = np.zeros(
            (self._max_iy - self._min_iy + 1, self._max_ix - self._min_ix + 1),
            dtype=np.int16)

        # 已被写入过的格子范围，发布时只发这一块，避免把整块预分配区域都塞进消息
        self._used: list[int] | None = None

        self._odom: deque[tuple[float, float, float]] = deque(maxlen=4000)
        self._frames = 0
        self._busy_seconds = 0.0        # 累计处理耗时，用于报告"单帧 ms"
        self._last_report = self.get_clock().now()

        self.create_subscription(
            PointCloud2, str(self.get_parameter('cloud_topic').value),
            self._on_cloud, 10)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._on_odom, 10)

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('map_topic').value), qos)

        rate = max(0.05, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'栅格分辨率 {self._res} m，高度带 [{self._z_min}, {self._z_max}] m，'
            f'清理距离上限 {self._max_clear_range} m；'
            f'静态障碍确认 {self._static_hits.confirmation_time:.1f}s '
            f'(最大观测间隔 {self._static_hits.max_gap:.1f}s)；'
            f'订阅 {self.get_parameter("cloud_topic").value} + '
            f'{self.get_parameter("odom_topic").value} -> '
            f'{self.get_parameter("map_topic").value}({self._map_frame})')

    # ---------------------------------------------------------------- 输入

    def _on_odom(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        self._odom.append((stamp, p.x, p.y))

    def _origin_at(self, stamp: float) -> tuple[float, float] | None:
        """取时间戳不晚于该帧的最后一个里程计位置，作为射线起点。"""
        if not self._odom:
            return None
        chosen = None
        for item in reversed(self._odom):
            if item[0] <= stamp:
                chosen = item
                break
        if chosen is None:
            chosen = self._odom[0]
        return chosen[1], chosen[2]

    def _on_cloud(self, msg: PointCloud2) -> None:
        try:
            pts = cloud_to_xyz(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=10.0)
            return
        if pts.shape[0] == 0:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        origin = self._origin_at(stamp)
        if origin is None:
            self.get_logger().warn(
                '还没收到里程计，无法确定射线起点，本帧丢弃。',
                throttle_duration_sec=10.0)
            return

        if pts.shape[0] > self._max_points:
            stride = int(np.ceil(pts.shape[0] / self._max_points))
            pts = pts[::stride]

        started = time.perf_counter()
        self._integrate(origin, pts, stamp)
        self._busy_seconds += time.perf_counter() - started
        self._frames += 1
        self._report()

    # ---------------------------------------------------------------- 累积

    def _integrate(self, origin: tuple[float, float], pts: np.ndarray,
                   stamp: float) -> None:
        ox, oy = origin
        dx = pts[:, 0].astype(np.float64) - ox
        dy = pts[:, 1].astype(np.float64) - oy
        rng = np.hypot(dx, dy)
        keep = rng > self._min_range
        if not keep.any():
            return
        pts, dx, dy = pts[keep], dx[keep], dy[keep]

        # 全部换算成"格子坐标"（浮点），原点 (0,0) 就是 map 帧原点
        oxf, oyf = ox / self._res, oy / self._res
        tx = pts[:, 0].astype(np.float64) / self._res
        ty = pts[:, 1].astype(np.float64) / self._res
        vx, vy = tx - oxf, ty - oyf

        # 射线推进的步数：到端点的格子数，但清理不超过 max_clear_range
        steps = np.maximum(np.maximum(np.abs(vx), np.abs(vy)), 1.0)
        clear_steps = np.minimum(steps, self._max_clear_range / self._res)
        n_iter = int(clear_steps.max())

        # 先把整帧的落空格子收集起来，最后一次性去重累加（见模块文档的性能说明）
        chunks = []
        for k in range(self._ray_stride, n_iter, self._ray_stride):
            active = clear_steps > k
            if not active.any():
                break
            t = k / steps[active]
            chunks.append(np.round(oxf + t * vx[active]).astype(np.int64))
            chunks.append(np.round(oyf + t * vy[active]).astype(np.int64))
        miss_ix = (np.concatenate(chunks[0::2]) if chunks
                   else np.empty(0, dtype=np.int64))
        miss_iy = (np.concatenate(chunks[1::2]) if chunks
                   else np.empty(0, dtype=np.int64))

        # 端点：只有落在高度带内才算障碍；带外的点仅参与上面的清理。
        # 命中格对本帧的 miss 免疫，否则薄墙/低矮障碍会被同一帧的其他射线刷掉。
        band = (pts[:, 2] >= self._z_min) & (pts[:, 2] <= self._z_max)
        hit_ix = (np.round(tx[band]).astype(np.int64) if band.any()
                  else np.empty(0, dtype=np.int64))
        hit_iy = (np.round(ty[band]).astype(np.int64) if band.any()
                  else np.empty(0, dtype=np.int64))

        if miss_ix.size == 0 and hit_ix.size == 0:
            return

        all_x = np.concatenate((miss_ix, hit_ix))
        all_y = np.concatenate((miss_iy, hit_iy))
        self._expand_to(int(all_x.min()), int(all_x.max()),
                        int(all_y.min()), int(all_y.max()))

        hit_flat = None
        if hit_ix.size:
            width = self._logodds.shape[1]
            hit_flat = (hit_iy - self._min_iy) * width + (hit_ix - self._min_ix)

        if miss_ix.size:
            self._apply(miss_ix, miss_iy, self._miss_inc,
                        protect_flat=hit_flat)
        if self._static_hits.confirmation_time > 0:
            # Confirm each cell once per sensor frame: dense returns from one
            # passing person are not many independent static observations.
            # Keep ALL hit endpoints protected from this frame's free rays,
            # including unconfirmed ones; never clear through an observed body.
            if hit_flat is not None:
                _, first, inverse = np.unique(
                    hit_flat, return_index=True, return_inverse=True)
                confirmed = self._static_hits.update(hit_ix[first], hit_iy[first], stamp)
                keep = confirmed[inverse]
                # Once confirmed, retain the established mapper's hit/miss
                # weights so intermittent thin walls are not weakened.
                hit_ix, hit_iy = hit_ix[keep], hit_iy[keep]
            else:
                self._static_hits.update(hit_ix, hit_iy, stamp)
        if hit_ix.size:
            self._apply(hit_ix, hit_iy, self._hit_inc)

    def _apply(self, ix: np.ndarray, iy: np.ndarray, inc: int,
               protect_flat: np.ndarray | None = None) -> None:
        """把一批格子更新按"出现次数 × 增量"一次性写入。

        整帧先去重再批量写入，而不是逐条射线 np.add.at：同一格子在一帧里会被
        多条射线穿到，用 np.unique 拿到出现次数能一次性累加完，省掉几十万次
        标量加法的开销。去重后的索引是唯一的，所以可以直接用花式赋值。

        protect_flat 中的格子会被跳过（用于"命中格免疫本帧 miss"）。
        """
        self._expand_to(int(ix.min()), int(ix.max()), int(iy.min()), int(iy.max()))
        width = self._logodds.shape[1]
        flat = (iy - self._min_iy) * width + (ix - self._min_ix)
        uniq, counts = np.unique(flat, return_counts=True)
        if protect_flat is not None and protect_flat.size:
            keep = ~np.isin(uniq, protect_flat)
            uniq = uniq[keep]
            counts = counts[keep]
        if uniq.size == 0:
            return
        if inc < 0 and self._static_hits.confirmation_time > 0:
            protected = (protect_flat if protect_flat is not None
                         else np.empty(0, dtype=np.int64))
            self._static_hits.clear(uniq % width + self._min_ix,
                                    uniq // width + self._min_iy,
                                    protected % width + self._min_ix,
                                    protected // width + self._min_iy)
        current = self._logodds.ravel()[uniq].astype(np.int64)
        updated = np.clip(current + counts * inc, self._lo_min, self._lo_max)
        self._logodds.ravel()[uniq] = updated.astype(np.int16)

        if self._used is None:
            self._used = [int(ix.min()), int(ix.max()), int(iy.min()), int(iy.max())]
        else:
            self._used[0] = min(self._used[0], int(ix.min()))
            self._used[1] = max(self._used[1], int(ix.max()))
            self._used[2] = min(self._used[2], int(iy.min()))
            self._used[3] = max(self._used[3], int(iy.max()))

    def _expand_to(self, min_x: int, max_x: int, min_y: int, max_y: int) -> None:
        """必要时扩张栅格，向四周留一格边距以免每帧都重新分配。"""
        if (min_x >= self._min_ix and max_x <= self._max_ix
                and min_y >= self._min_iy and max_y <= self._max_iy):
            return
        margin = 32
        new_min_ix = min(self._min_ix, min_x - margin)
        new_max_ix = max(self._max_ix, max_x + margin)
        new_min_iy = min(self._min_iy, min_y - margin)
        new_max_iy = max(self._max_iy, max_y + margin)

        grown = np.zeros((new_max_iy - new_min_iy + 1, new_max_ix - new_min_ix + 1),
                         dtype=np.int16)
        dy = self._min_iy - new_min_iy
        dx = self._min_ix - new_min_ix
        old_h, old_w = self._logodds.shape
        grown[dy:dy + old_h, dx:dx + old_w] = self._logodds

        self._logodds = grown
        self._min_ix, self._max_ix = new_min_ix, new_max_ix
        self._min_iy, self._max_iy = new_min_iy, new_max_iy
        self.get_logger().info(
            f'栅格扩张到 {new_max_ix - new_min_ix + 1} × {new_max_iy - new_min_iy + 1} 格')

    # ---------------------------------------------------------------- 输出

    def _publish(self) -> None:
        if self._used is None:
            return
        x0, x1, y0, y1 = self._used
        sub = self._logodds[y0 - self._min_iy:y1 - self._min_iy + 1,
                            x0 - self._min_ix:x1 - self._min_ix + 1]

        out = np.full(sub.shape, -1, dtype=np.int8)      # 未知
        out[sub <= self._free_thresh] = 0                # 空闲
        out[sub >= self._occ_thresh] = 100               # 占据

        msg = OccupancyGrid()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self._res
        msg.info.width = out.shape[1]
        msg.info.height = out.shape[0]
        msg.info.origin.position.x = float(x0 * self._res)
        msg.info.origin.position.y = float(y0 * self._res)
        msg.info.origin.orientation.w = 1.0
        # 不能直接赋 bytes：bytes 迭代出来是无符号的 0~255，而 rclpy 对 int8[] 的
        # 校验要求 -128~127，未知格 -1 会被判成 255 而断言失败。
        # array.array('b') 的 frombytes 是原样拷贝比特，且正好命中 setter 的快速路径。
        data = array.array('b')
        data.frombytes(out.tobytes())
        msg.data = data
        self._pub.publish(msg)

    def _report(self) -> None:
        now = self.get_clock().now()
        if (now - self._last_report).nanoseconds < 10e9:
            return
        self._last_report = now
        if self._used is None:
            return
        x0, x1, y0, y1 = self._used
        sub = self._logodds[y0 - self._min_iy:y1 - self._min_iy + 1,
                            x0 - self._min_ix:x1 - self._min_ix + 1]
        occ = int((sub >= self._occ_thresh).sum())
        free = int((sub <= self._free_thresh).sum())
        size = sub.shape[0] * sub.shape[1]
        avg_ms = 1000.0 * self._busy_seconds / max(1, self._frames)
        self.get_logger().info(
            f'{self._frames} 帧 | 栅格 {sub.shape[1]}×{sub.shape[0]} '
            f'({size * self._res * self._res:.0f} m²) | '
            f'占据 {occ} 空闲 {free} 未知 {size - occ - free} | '
            f'单帧 {avg_ms:.1f} ms')


def main() -> None:
    rclpy.init()
    node = GridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
