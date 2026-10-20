"""航点文件读取、别名解析与语音播报文本生成。"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml


_CN_NUMERALS = ["零", "一", "二", "三", "四", "五", "六", "七", "八", "九", "十"]


def _cn_numeral(value: int) -> str:
    if 0 <= value <= 10:
        return _CN_NUMERALS[value]
    if value < 20:
        return "十" + _CN_NUMERALS[value % 10]
    if value < 100:
        tens, ones = divmod(value, 10)
        return _CN_NUMERALS[tens] + "十" + (_CN_NUMERALS[ones] if ones else "")
    return str(value)


def normalize_text(text: str) -> str:
    """归一化点位名称/口令片段，便于比较。"""
    text = str(text).strip().lower()
    text = re.sub(r"[\s_\-，。！？、：:；;\"'“”‘’()（）]+", "", text)
    # 把中文数词换成阿拉伯数字，覆盖“一号点/第二个点/十号点”
    for value in range(10, -1, -1):
        text = text.replace(_cn_numeral(value), str(value))
    return text


@dataclass(frozen=True)
class Waypoint:
    id: int
    name: str
    x: float
    y: float
    yaw: float = 0.0
    aliases: tuple[str, ...] = field(default_factory=tuple)

    def generated_aliases(self) -> tuple[str, ...]:
        cn = _cn_numeral(self.id)
        values = {
            str(self.id),
            f"{self.id}号",
            f"{self.id}号点",
            f"{self.id}号点位",
            f"点{self.id}",
            f"点位{self.id}",
            f"第{self.id}个点",
            f"第{self.id}个点位",
            f"{cn}号点",
            f"{cn}号点位",
            f"第{cn}个点",
            f"point_{self.id}",
            f"point{self.id}",
            f"point {self.id}",
            self.name,
            *self.aliases,
        }
        return tuple(v for v in values if v)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "aliases": list(self.aliases),
        }


class WaypointStore:
    """从 YAML 读取点位，并把“1号点位/一号点/point_1/厨房”解析成 Waypoint。"""

    def __init__(self, path: str):
        self.path = path
        self.frame = "map"
        self.points: List[Waypoint] = []
        self.load_error: Optional[str] = None
        self.load()

    def load(self) -> None:
        self.points = []
        self.load_error = None
        if not self.path or not os.path.isfile(self.path):
            self.load_error = f"航点文件不存在: {self.path}"
            return
        try:
            with open(self.path, encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            self.frame = str(data.get("frame") or "map")
            for item in data.get("points") or []:
                self.points.append(
                    Waypoint(
                        id=int(item["id"]),
                        name=str(item.get("name") or f"point_{item['id']}"),
                        x=float(item["x"]),
                        y=float(item["y"]),
                        yaw=float(item.get("yaw") or 0.0),
                        aliases=tuple(str(a) for a in (item.get("aliases") or [])),
                    )
                )
            self.points.sort(key=lambda point: point.id)
        except Exception as exc:  # 文件坏了也要让节点起来并给出可读错误
            self.load_error = f"航点文件解析失败: {exc}"

    def resolve(self, query: str) -> Optional[Waypoint]:
        if not query and query != 0:
            return None
        normalized = normalize_text(query)
        if not normalized:
            return None

        # 1) 完全匹配任意别名
        for point in self.points:
            for alias in point.generated_aliases():
                if normalized == normalize_text(alias):
                    return point

        # 2) 查询里带数字：优先按 id 匹配（覆盖“去1号点位”“第2个点”）
        digit_match = re.search(r"\d+", normalized)
        if digit_match:
            point_id = int(digit_match.group(0))
            for point in self.points:
                if point.id == point_id:
                    return point

        # 3) 名称包含匹配（支持“厨房”“门口”这类自定义别名）
        candidates = []
        for point in self.points:
            for alias in point.generated_aliases():
                normalized_alias = normalize_text(alias)
                if len(normalized_alias) < 2:
                    continue
                if normalized_alias in normalized or (
                    len(normalized) >= 2 and normalized in normalized_alias
                ):
                    candidates.append((len(normalized_alias), point))
        if candidates:
            candidates.sort(key=lambda item: item[0], reverse=True)
            return candidates[0][1]
        return None

    def list_text(self) -> str:
        if not self.points:
            return "当前没有配置任何点位。"
        names = []
        for point in self.points:
            label = f"{point.id}号点位"
            display = point.aliases[0] if point.aliases else point.name
            if display and display != f"point_{point.id}":
                label += f"（{display}）"
            names.append(label)
        return "、".join(names)

    def as_dicts(self) -> List[Dict[str, Any]]:
        return [point.as_dict() for point in self.points]
