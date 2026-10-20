#!/usr/bin/env python3
"""Native ROS 2 launch manager for the Lidar_nav2 workspace."""

import json
import os
import shlex
import signal
import subprocess
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from xml.etree import ElementTree


WORKSPACE = Path(__file__).resolve().parent.parent
STATE_DIR = Path("/tmp/lidar_nav2_launch_manager")


@dataclass(frozen=True)
class Task:
    task_id: str
    group: str
    name: str
    command: str
    description: str
    risk: str


TASKS = (
    Task("bunker", "底盘", "BUNKER MINI 2.0", "ros2 launch me_nav2_bringup bunker_mini2_base.launch.py", "CAN 底盘驱动，发布 /wheel_odom", "需要 CAN"),
    Task("livox", "实机", "Livox Fast-LIO 驱动", "ros2 launch livox_ros_driver2 fast_lio_msg_MID360_launch.py rviz:=false", "MID360 到 FAST-LIO 的消息接口", "实机"),
    Task("fastlio", "实机", "FAST-LIO", "ros2 launch fast_lio mapping.launch.py use_sim_time:=false rviz:=false", "激光惯性里程计", "实机"),
    Task("lio_interface", "实机", "FAST-LIO 接口", "ros2 launch lio_interface fastlio_lio_interface_launch.py use_sim_time:=false", "发布配准点云和里程计接口", "实机"),
    Task("robot_description", "通用", "机器人描述", "ros2 launch gld_robot_description gld_robot_description_launch.py rviz:=false", "发布 base_footprint 到传感器的固定 TF", "安全"),
    Task("scan_generation", "通用", "扫描生成", "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py use_sim_time:=false", "生成 /registered_scan 和 LIO 里程计 TF", "安全"),
    Task("laserscan", "通用", "点云转 2D 激光", "ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py use_sim_time:=false", "将 3D 点云切片为 /scan", "安全"),
    Task("relocalization", "定位", "GICP 重定位", "ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py", "唯一的 map 到 odom 发布者", "定位"),
    Task("nav2_real", "导航", "Nav2 实机 + 安全层", "ros2 launch nav2_safety safe_nav_bringup_launch.py use_sim_time:=false", "导航控制，发布 /cmd_vel（经安全层仲裁后到 cmd_vel_deadman）", "会控制底盘"),
    Task("slam", "建图", "SLAM Toolbox 实机", "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$WORKSPACE_ROOT/src/me_nav2_bringup/config/slam_toolbox_params.yaml use_sim_time:=false", "2D 建图", "实机"),
)


class LaunchManager(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("NavStack Launch Console")
        self.geometry("1280x780")
        self.minsize(940, 600)
        self.selected_group = tk.StringVar(value="全部")
        self.query = tk.StringVar()
        self.status = tk.StringVar(value="本地任务服务已就绪")
        self.command_overrides = self.load_command_overrides()
        self.custom_tasks = self.load_custom_tasks()
        self.hidden_tasks = self.load_hidden_tasks()
        self._configure_style()
        self._build()
        self.query.trace_add("write", lambda *_: self.refresh())
        self.refresh()
        self.after(1000, self.poll)

    def _configure_style(self):
        self.configure(bg="#eef2f4")
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("Treeview", background="#ffffff", fieldbackground="#ffffff", foreground="#253542", rowheight=38, font=("Noto Sans CJK SC", 10))
        style.configure("Treeview.Heading", background="#e8eef1", foreground="#263946", font=("Noto Sans CJK SC", 10, "bold"), relief="flat")
        style.map("Treeview", background=[("selected", "#d8f0ea")], foreground=[("selected", "#123d36")])
        style.configure("TButton", padding=(10, 6), font=("Noto Sans CJK SC", 10))
        style.configure("Accent.TButton", background="#168a75", foreground="#ffffff")
        style.map("Accent.TButton", background=[("active", "#107463")])
        style.configure("Danger.TButton", background="#bd4b4b", foreground="#ffffff")
        style.map("Danger.TButton", background=[("active", "#a73f3f")])

    def _build(self):
        header = tk.Frame(self, bg="#10293d", height=76)
        header.pack(fill="x")
        header.pack_propagate(False)
        tk.Label(header, text="NavStack", bg="#10293d", fg="#ffffff", font=("Noto Sans CJK SC", 19, "bold")).pack(side="left", padx=(24, 10), pady=18)
        tk.Label(header, text="LAUNCH CONSOLE", bg="#10293d", fg="#8cb2c1", font=("Noto Sans CJK SC", 10, "bold")).pack(side="left", pady=22)
        ttk.Button(header, text="刷新状态", command=self.refresh).pack(side="right", padx=22, pady=18)

        filters = tk.Frame(self, bg="#eef2f4")
        filters.pack(fill="x", padx=24, pady=(20, 10))
        tk.Label(filters, text="任务控制中心", bg="#eef2f4", fg="#172934", font=("Noto Sans CJK SC", 18, "bold")).pack(side="left")
        ttk.Entry(filters, textvariable=self.query, width=34).pack(side="right")
        tk.Label(filters, text="筛选", bg="#eef2f4", fg="#61727f").pack(side="right", padx=(0, 8))

        body = tk.Frame(self, bg="#eef2f4")
        body.pack(fill="both", expand=True, padx=24, pady=8)
        sidebar = tk.Frame(body, bg="#ffffff", width=135, highlightbackground="#d7e0e4", highlightthickness=1)
        sidebar.pack(side="left", fill="y", padx=(0, 14))
        sidebar.pack_propagate(False)
        tk.Label(sidebar, text="任务分类", bg="#ffffff", fg="#657681", font=("Noto Sans CJK SC", 10, "bold")).pack(anchor="w", padx=14, pady=(16, 8))
        for group in ("全部", "自定义", "底盘", "实机", "通用", "定位", "导航", "建图"):
            ttk.Radiobutton(sidebar, text=group, value=group, variable=self.selected_group, command=self.refresh).pack(anchor="w", padx=14, pady=4)

        main = tk.Frame(body, bg="#ffffff", highlightbackground="#d7e0e4", highlightthickness=1)
        main.pack(side="left", fill="both", expand=True)
        columns = ("name", "group", "status", "risk", "command")
        self.tree = ttk.Treeview(main, columns=columns, show="headings", selectmode="browse")
        for key, title, width in (("name", "任务", 190), ("group", "分类", 80), ("status", "状态", 95), ("risk", "风险", 100), ("command", "启动命令", 570)):
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, anchor="w", stretch=key == "command")
        self.tree.pack(fill="both", expand=True, padx=12, pady=12)
        self.tree.bind("<<TreeviewSelect>>", lambda _: self._show_detail())
        self.tree.bind("<Double-1>", self.start_on_double_click)

        actions = tk.Frame(main, bg="#ffffff")
        actions.pack(fill="x", padx=12, pady=(0, 12))
        self.detail = tk.StringVar(value="选择一个任务查看说明")
        tk.Label(actions, textvariable=self.detail, bg="#ffffff", fg="#637682", anchor="w").pack(side="left", fill="x", expand=True)
        ttk.Button(actions, text="停止", style="Danger.TButton", command=self.stop_selected).pack(side="right", padx=(8, 0))
        ttk.Button(actions, text="在新终端启动", style="Accent.TButton", command=self.start_selected).pack(side="right")
        ttk.Button(actions, text="编辑启动命令", command=self.edit_selected_command).pack(side="right", padx=(0, 8))
        ttk.Button(actions, text="删除自定义任务", command=self.delete_selected_task).pack(side="right", padx=(0, 8))
        ttk.Button(actions, text="添加 Launch", style="Accent.TButton", command=self.add_launch_task).pack(side="right", padx=(0, 8))

        footer = tk.Label(self, textvariable=self.status, anchor="w", bg="#dfe9ed", fg="#385461", padx=24, pady=8)
        footer.pack(fill="x", side="bottom")

    def state_file(self, task: Task) -> Path:
        return STATE_DIR / f"python_{task.task_id}.pid"

    @property
    def command_file(self) -> Path:
        return STATE_DIR / "commands.json"

    @property
    def custom_tasks_file(self) -> Path:
        return STATE_DIR / "custom_tasks.json"

    @property
    def hidden_tasks_file(self) -> Path:
        return STATE_DIR / "hidden_tasks.json"

    def load_hidden_tasks(self) -> set[str]:
        """已删除的内置任务 id 列表，避免重启后“复活”。"""
        try:
            data = json.loads(self.hidden_tasks_file.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return set()
        if not isinstance(data, list):
            return set()
        return {item for item in data if isinstance(item, str) and item.strip()}

    def save_hidden_tasks(self):
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        temporary = self.hidden_tasks_file.with_suffix(".tmp")
        temporary.write_text(json.dumps(sorted(self.hidden_tasks), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.hidden_tasks_file)

    def load_command_overrides(self) -> dict[str, str]:
        try:
            data = json.loads(self.command_file.read_text(encoding="utf-8"))
            return {task_id: command for task_id, command in data.items() if isinstance(task_id, str) and isinstance(command, str) and command.strip()}
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return {}

    def save_command_overrides(self):
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        temporary = self.command_file.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.command_overrides, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.command_file)

    def load_custom_tasks(self) -> list[Task]:
        try:
            data = json.loads(self.custom_tasks_file.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return []
        if not isinstance(data, list):
            return []
        tasks = []
        for item in data:
            if not isinstance(item, dict):
                continue
            fields = ("task_id", "group", "name", "command", "description", "risk")
            if all(isinstance(item.get(field), str) and item[field].strip() for field in fields) and item["task_id"].startswith("custom_"):
                tasks.append(Task(**{field: item[field] for field in fields}))
        return tasks

    def save_custom_tasks(self):
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        temporary = self.custom_tasks_file.with_suffix(".tmp")
        data = [{"task_id": task.task_id, "group": task.group, "name": task.name, "command": task.command, "description": task.description, "risk": task.risk} for task in self.custom_tasks]
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.custom_tasks_file)

    def all_tasks(self):
        return (*(task for task in TASKS if task.task_id not in self.hidden_tasks), *self.custom_tasks)

    def command_for(self, task: Task) -> str:
        return self.command_overrides.get(task.task_id, task.command)

    def pid_for(self, task: Task):
        try:
            pid = int(self.state_file(task).read_text().strip())
            os.kill(pid, 0)
            return pid
        except (FileNotFoundError, ValueError, ProcessLookupError):
            self.state_file(task).unlink(missing_ok=True)
            return None
        except PermissionError:
            return None

    def visible_tasks(self):
        needle = self.query.get().casefold().strip()
        return [task for task in self.all_tasks() if (self.selected_group.get() == "全部" or task.group == self.selected_group.get()) and (not needle or needle in f"{task.name} {self.command_for(task)} {task.description}".casefold())]

    def refresh(self):
        selected = self.tree.selection()
        selected_id = selected[0] if selected else None
        for item in self.tree.get_children():
            self.tree.delete(item)
        running = 0
        for task in self.visible_tasks():
            pid = self.pid_for(task)
            if pid:
                running += 1
            status = f"● 运行中 ({pid})" if pid else "● 已停止"
            self.tree.insert("", "end", iid=task.task_id, values=(task.name, task.group, status, task.risk, self.command_for(task)), tags=("running" if pid else "stopped",))
        if selected_id and self.tree.exists(selected_id):
            self.tree.selection_set(selected_id)
        self.status.set(f"本地桌面服务正常 | 当前显示 {len(self.visible_tasks())} 项任务，运行中 {running} 项")
        self.tree.tag_configure("running", foreground="#168a75")
        self.tree.tag_configure("stopped", foreground="#9a4d4d")

    def selected_task(self):
        selected = self.tree.selection()
        if not selected:
            messagebox.showinfo("任务控制", "请先选择一个任务。", parent=self)
            return None
        return next(task for task in self.all_tasks() if task.task_id == selected[0])

    def _show_detail(self):
        task = self.selected_task_silent()
        if task:
            self.detail.set(f"{task.description}    {self.command_for(task)}")

    def start_on_double_click(self, _event):
        task = self.selected_task_silent()
        if task and not self.pid_for(task):
            self.start_selected()

    def edit_selected_command(self):
        task = self.selected_task()
        if not task:
            return
        dialog = tk.Toplevel(self)
        dialog.title(f"编辑启动命令 - {task.name}")
        dialog.transient(self)
        dialog.grab_set()
        dialog.geometry("820x260")
        dialog.minsize(560, 220)
        frame = tk.Frame(dialog, padx=18, pady=16)
        frame.pack(fill="both", expand=True)
        tk.Label(frame, text=task.name, font=("Noto Sans CJK SC", 13, "bold")).pack(anchor="w")
        tk.Label(frame, text="命令将在下次启动时生效，可使用 $WORKSPACE_ROOT。", fg="#637682").pack(anchor="w", pady=(4, 10))
        editor = tk.Text(frame, height=5, wrap="word", font=("Monospace", 10))
        editor.pack(fill="both", expand=True)
        editor.insert("1.0", self.command_for(task))
        buttons = tk.Frame(frame)
        buttons.pack(fill="x", pady=(12, 0))

        def restore_default():
            editor.delete("1.0", "end")
            editor.insert("1.0", task.command)

        def save():
            command = editor.get("1.0", "end").strip()
            if not command:
                messagebox.showerror("启动命令不能为空", "请输入有效的启动命令。", parent=dialog)
                return
            if command == task.command:
                self.command_overrides.pop(task.task_id, None)
            else:
                self.command_overrides[task.task_id] = command
            try:
                self.save_command_overrides()
            except OSError as error:
                messagebox.showerror("保存失败", str(error), parent=dialog)
                return
            dialog.destroy()
            self.status.set(f"已保存 {task.name} 的启动命令")
            self.refresh()

        ttk.Button(buttons, text="恢复默认", command=restore_default).pack(side="left")
        ttk.Button(buttons, text="取消", command=dialog.destroy).pack(side="right")
        ttk.Button(buttons, text="保存", style="Accent.TButton", command=save).pack(side="right", padx=(0, 8))

    def add_launch_task(self):
        dialog = tk.Toplevel(self)
        dialog.title("添加 Launch 任务")
        dialog.transient(self)
        dialog.grab_set()
        dialog.geometry("620x420")
        dialog.minsize(540, 390)
        frame = tk.Frame(dialog, padx=20, pady=18)
        frame.pack(fill="both", expand=True)
        tk.Label(frame, text="添加 ROS Launch 任务", font=("Noto Sans CJK SC", 14, "bold")).grid(row=0, column=0, columnspan=2, sticky="w")
        tk.Label(frame, text="登记已有 launch 文件；启动时将在新终端执行。", fg="#637682").grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 16))
        values = {key: tk.StringVar() for key in ("name", "package", "launch", "args", "description")}
        group = tk.StringVar(value="自定义")
        risk = tk.StringVar(value="安全")
        fields = (("任务名称", "name"), ("ROS 包名", "package"), ("Launch 文件", "launch"), ("附加参数（可选）", "args"), ("说明", "description"))
        for row, (label, key) in enumerate(fields, start=2):
            tk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=5)
            ttk.Entry(frame, textvariable=values[key], width=55).grid(row=row, column=1, sticky="ew", pady=5)
        def choose_launch_file():
            selected_path = filedialog.askopenfilename(
                parent=dialog,
                title="选择 ROS Launch 文件",
                initialdir=WORKSPACE / "src",
                filetypes=(("ROS Launch", "*.launch.py *.launch.xml *.launch.yaml"), ("所有文件", "*")),
            )
            if not selected_path:
                return
            launch_path = Path(selected_path)
            values["launch"].set(launch_path.name)
            for parent in (launch_path.parent, *launch_path.parents):
                package_xml = parent / "package.xml"
                if not package_xml.is_file():
                    continue
                try:
                    package_name = ElementTree.parse(package_xml).getroot().findtext("name", default="").strip()
                except ElementTree.ParseError:
                    package_name = ""
                if package_name:
                    values["package"].set(package_name)
                break
            if not values["name"].get().strip():
                values["name"].set(launch_path.stem.replace(".launch", ""))
        ttk.Button(frame, text="选择文件", command=choose_launch_file).grid(row=4, column=2, padx=(8, 0), pady=5)
        tk.Label(frame, text="分类").grid(row=7, column=0, sticky="w", pady=5)
        ttk.Combobox(frame, textvariable=group, values=("自定义", "底盘", "实机", "通用", "定位", "导航", "建图"), state="readonly", width=20).grid(row=7, column=1, sticky="w", pady=5)
        tk.Label(frame, text="风险").grid(row=8, column=0, sticky="w", pady=5)
        ttk.Combobox(frame, textvariable=risk, values=("安全", "实机", "定位", "需要 CAN", "会控制底盘"), state="readonly", width=20).grid(row=8, column=1, sticky="w", pady=5)
        frame.columnconfigure(1, weight=1)

        def save():
            name, package, launch = (values[key].get().strip() for key in ("name", "package", "launch"))
            if not all((name, package, launch)):
                messagebox.showerror("信息不完整", "请填写任务名称、ROS 包名和 Launch 文件。", parent=dialog)
                return
            if not launch.endswith((".launch.py", ".launch.xml", ".launch.yaml")):
                if not messagebox.askyesno("确认文件名", "Launch 文件通常以 .launch.py、.launch.xml 或 .launch.yaml 结尾，仍要添加吗？", parent=dialog):
                    return
            task_id = f"custom_{int(time.time() * 1000)}"
            args = values["args"].get().strip()
            command = f"ros2 launch {package} {launch}" + (f" {args}" if args else "")
            self.custom_tasks.append(Task(task_id, group.get(), name, command, values["description"].get().strip() or "自定义 Launch 任务", risk.get()))
            try:
                self.save_custom_tasks()
            except OSError as error:
                self.custom_tasks.pop()
                messagebox.showerror("保存失败", str(error), parent=dialog)
                return
            dialog.destroy()
            self.selected_group.set("全部")
            self.status.set(f"已添加 Launch 任务 {name}")
            self.refresh()

        buttons = tk.Frame(frame)
        buttons.grid(row=9, column=0, columnspan=2, sticky="ew", pady=(18, 0))
        ttk.Button(buttons, text="取消", command=dialog.destroy).pack(side="right")
        ttk.Button(buttons, text="添加", style="Accent.TButton", command=save).pack(side="right", padx=(0, 8))

    def delete_selected_task(self):
        task = self.selected_task()
        if not task:
            return
        if self.pid_for(task):
            messagebox.showwarning("任务正在运行", "请先停止任务，再删除其登记项。", parent=self)
            return
        prompt = f"确定从管理器中删除“{task.name}”吗？\n不会删除磁盘上的 launch 文件。"
        if task.task_id.startswith("custom_"):
            if not messagebox.askyesno("删除自定义任务", prompt, parent=self):
                return
            self.custom_tasks = [item for item in self.custom_tasks if item.task_id != task.task_id]
            self.save_custom_tasks()
        else:
            builtin_prompt = prompt + "\n\n内置任务删除后会持久记录（重启管理器不会恢复）。"
            if not messagebox.askyesno("删除内置任务", builtin_prompt, parent=self):
                return
            self.hidden_tasks.add(task.task_id)
            self.save_hidden_tasks()
        self.command_overrides.pop(task.task_id, None)
        try:
            self.save_command_overrides()
        except OSError as error:
            messagebox.showerror("删除失败", str(error), parent=self)
            return
        self.status.set(f"已删除任务 {task.name}")
        self.detail.set("选择一个任务查看说明")
        self.refresh()

    def selected_task_silent(self):
        selected = self.tree.selection()
        return next((task for task in self.all_tasks() if selected and task.task_id == selected[0]), None)

    def start_selected(self):
        task = self.selected_task()
        if not task or self.pid_for(task):
            if task:
                self.status.set(f"{task.name} 已在运行")
            return
        if task.task_id == "bunker" and not Path("/sys/class/net/can0").exists():
            messagebox.showerror("CAN 接口不可用", "系统未检测到 can0。请先连接并配置 CAN 适配器，再启动 BUNKER 底盘驱动。", parent=self)
            return
        if task.risk in ("需要 CAN", "会控制底盘") and not messagebox.askyesno("确认启动", f"{task.name} 可能连接或控制实机，确认启动？", parent=self):
            return
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        state_file = self.state_file(task)
        command = self.command_for(task).replace("$WORKSPACE_ROOT", str(WORKSPACE))
        quoted_state = shlex.quote(str(state_file))
        quoted_root = shlex.quote(str(WORKSPACE))
        wrapper = (
            f"STATE_FILE={quoted_state}; cleanup() {{ rm -f \"$STATE_FILE\"; }}; "
            "forward() { [ -n \"${child:-}\" ] && kill -INT -- \"-$child\" 2>/dev/null || true; }; "
            f"trap forward INT TERM; trap cleanup EXIT; cd {quoted_root}; source install/setup.bash; "
            f"setsid bash -lc {shlex.quote('exec ' + command)} & child=$!; "
            "printf '%s\\n' \"$child\" > \"$STATE_FILE\"; wait \"$child\"; status=$?; "
            "echo; echo \"任务结束，退出码: $status\"; read -r -p \"按 Enter 关闭终端...\"; exit \"$status\""
        )
        try:
            subprocess.Popen(["gnome-terminal", f"--title={task.name}", "--", "bash", "-lc", wrapper], start_new_session=True)
        except FileNotFoundError:
            messagebox.showerror("无法打开终端", "未找到 gnome-terminal。", parent=self)
            return
        self.status.set(f"已在新终端启动 {task.name}")
        self.after(400, self.refresh)

    def stop_selected(self):
        task = self.selected_task()
        if not task:
            return
        pid = self.pid_for(task)
        if not pid:
            self.status.set(f"{task.name} 未运行")
            return
        try:
            os.killpg(pid, signal.SIGINT)
            self.status.set(f"已请求停止 {task.name}")
        except ProcessLookupError:
            self.state_file(task).unlink(missing_ok=True)
            self.status.set(f"{task.name} 已退出")
        self.after(500, self.refresh)

    def poll(self):
        self.refresh()
        self.after(1000, self.poll)


if __name__ == "__main__":
    LaunchManager().mainloop()
