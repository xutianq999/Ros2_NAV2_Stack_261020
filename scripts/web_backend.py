#!/usr/bin/env python3
"""BUNKER 导航工作台后端（第一版）。

职责：
  - 托管 src/web 静态页面（和 API 同源，避免 CORS）
  - 点位持久化：读写当前工作区/巡检方案的点位表（默认工作区镜像一份到 config/waypoints.yaml）
  - 任务持久化：SQLite（web_backend_data/missions.db）
  - 服务端任务执行：通过 rclpy NavigateToPose action 发目标，关掉浏览器任务也不断
  - 地图保存：调用 nav2_map_server/map_saver_cli
  - PCD 转栅格：调用 scripts/pcd_to_occupancy_map.py
  - 只读聚合 /robot/status 供页面查询

运行（必须 source ROS 和本工作空间）：
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 scripts/web_backend.py --host 0.0.0.0 --port 5500

注意：
  - 默认没有鉴权，只适合可信局域网；设置 --token 或 BUNKER_WEB_TOKEN 后
    /api/* 需要 Authorization: Bearer <token>（静态页面仍可打开，页面会弹登录框）。
  - 服务端任务执行使用 NavigateToPose action；未启动 Nav2 时任务会失败并给出原因。
  - 3D PCD 由 Super-LIO 在退出建图时保存，本后端不会强行停止建图进程。
"""

from __future__ import annotations

import argparse
import io
import json
import math
import os
import re
import secrets
import shutil
import sqlite3
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import yaml
from fastapi import FastAPI, File, HTTPException, Request, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

# ---------------------------------------------------------------------------
# 路径
# ---------------------------------------------------------------------------
WORKSPACE = Path(__file__).resolve().parent.parent
WEB_ROOT = WORKSPACE / 'src' / 'web'
POINTS_FILE = WORKSPACE / 'src' / 'me_nav2_bringup' / 'config' / 'waypoints.yaml'
ANNOTATIONS_FILE = WORKSPACE / 'src' / 'me_nav2_bringup' / 'config' / 'map_annotations.yaml'
MAP_CONFIG_FILE = WORKSPACE / 'src' / 'me_nav2_bringup' / 'config' / 'map_config.yaml'
MAP_DIR = WORKSPACE / 'src' / 'me_nav2_bringup' / 'map'
DATA_DIR = WORKSPACE / 'web_backend_data'
UPLOAD_MAP_DIR = DATA_DIR / 'map_uploads'
WORKSPACES_DIR = DATA_DIR / 'workspaces'
ACTIVE_WORKSPACE_FILE = DATA_DIR / 'active_workspace.json'
DB_PATH = DATA_DIR / 'missions.db'
RUNTIME_FILE = DATA_DIR / 'runtime_mission.json'
NAME_RE = re.compile(r'^[A-Za-z0-9_.-]{1,64}$')

# 在 MissionStore 之后初始化；点位/标注读写通过它确定当前工作区文件。
workspace_store: Optional['WorkspaceStore'] = None

# 空字符串 = 不启用鉴权（默认，适合可信局域网）。
AUTH_TOKEN = ''


def _token_from_request(request: Request) -> str:
    header = request.headers.get('authorization', '')
    if header.lower().startswith('bearer '):
        return header[7:].strip()
    return request.headers.get('x-auth-token', '').strip()


def _token_ok(candidate: str) -> bool:
    return bool(AUTH_TOKEN) and secrets.compare_digest(candidate, AUTH_TOKEN)


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def _rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(WORKSPACE))
    except ValueError:
        return str(path)


def _finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise HTTPException(status_code=422, detail=f'{name} 必须是数字')
    if not math.isfinite(number):
        raise HTTPException(status_code=422, detail=f'{name} 必须是有限数字')
    return number


def _check_name(name: str) -> str:
    """用于文件名/地图名：只允许 ASCII 安全字符。"""
    name = (name or '').strip()
    if not NAME_RE.match(name):
        raise HTTPException(status_code=422, detail='名称只能包含字母、数字、_ . -，长度 1-64')
    return name


def _check_text(name: str, field: str = '名称') -> str:
    """用于任务/点位显示名：允许中文，但限制长度。"""
    name = (name or '').strip()
    if not name or len(name) > 64:
        raise HTTPException(status_code=422, detail=f'{field}不能为空且最长 64 个字符')
    return name


def _safe_path(path: str | Path, base: Path) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute():
        candidate = WORKSPACE / candidate
    candidate = candidate.resolve()
    try:
        candidate.relative_to(base.resolve())
    except ValueError:
        raise HTTPException(status_code=422, detail=f'路径必须位于 {base} 内')
    return candidate


def _persist_runtime(state: dict) -> None:
    """把当前任务执行状态写到磁盘，后端重启后可以恢复/标记中断。"""
    try:
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        tmp = RUNTIME_FILE.with_suffix('.tmp')
        tmp.write_text(json.dumps(state, ensure_ascii=False), encoding='utf-8')
        tmp.replace(RUNTIME_FILE)
    except OSError:
        pass


def _load_runtime() -> Optional[dict]:
    try:
        data = json.loads(RUNTIME_FILE.read_text(encoding='utf-8'))
        return data if isinstance(data, dict) else None
    except (OSError, ValueError, TypeError):
        return None


# ---------------------------------------------------------------------------
# 点位存储
# ---------------------------------------------------------------------------
class PointModel(BaseModel):
    id: Optional[int] = None
    name: str = Field(min_length=1, max_length=64)
    x: float
    y: float
    yaw: float = 0.0
    dwell: float = Field(default=0.0, ge=0.0, le=3600.0)

    def normalised(self) -> dict:
        return {
            'id': self.id,
            'name': self.name.strip(),
            'x': _finite(self.x, 'x'),
            'y': _finite(self.y, 'y'),
            'yaw': _finite(self.yaw, 'yaw'),
            'dwell': round(_finite(self.dwell, 'dwell'), 3),
        }


class PointsPayload(BaseModel):
    frame: str = 'map'
    points: list[PointModel] = Field(default_factory=list)


def _active_ws() -> Optional[dict]:
    if workspace_store is None:
        return None
    try:
        return workspace_store.active()
    except Exception:
        return None


def _scoped_file(filename: str, legacy: Path) -> Path:
    """当前工作区目录下的文件；没有工作区上下文时才回退到旧全局文件。"""
    ws = _active_ws()
    if ws is None:
        return legacy
    path = Path(ws['directory']) / filename
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _active_points_file() -> Path:
    """当前工作区的当前巡检方案点位表。"""
    ws = _active_ws()
    if ws is not None and workspace_store is not None:
        plan = workspace_store.active_plan(ws['id'])
        if plan is not None:
            return Path(plan['points_file'])
    return POINTS_FILE


def _mirror_legacy_points(data: dict) -> None:
    """默认工作区默认方案写一份镜像到旧的 config/waypoints.yaml。"""
    ws = _active_ws()
    if ws is None or not ws.get('is_default') or workspace_store is None:
        return
    plan = workspace_store.active_plan(ws['id'])
    if plan is None or not plan.get('is_default'):
        return
    try:
        if Path(plan['points_file']).resolve() == POINTS_FILE.resolve():
            return
        content = yaml.safe_dump(data, allow_unicode=True, sort_keys=False)
        POINTS_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = POINTS_FILE.with_suffix('.yaml.tmp')
        tmp.write_text(content, encoding='utf-8')
        tmp.replace(POINTS_FILE)
    except OSError:
        pass


def read_points() -> dict:
    points_file = _active_points_file()
    if not points_file.exists():
        return {'frame': 'map', 'points': []}
    with points_file.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    points = []
    for index, raw in enumerate(data.get('points') or [], start=1):
        try:
            points.append({
                'id': int(raw.get('id', index)),
                'name': str(raw.get('name', f'point_{index}')),
                'x': float(raw['x']),
                'y': float(raw['y']),
                'yaw': float(raw.get('yaw', 0.0)),
                'dwell': float(raw.get('dwell', 0.0)),
            })
        except (KeyError, TypeError, ValueError):
            continue
    return {'frame': str(data.get('frame', 'map')), 'points': points}


def _validate_points(points: list[dict]) -> list[dict]:
    names = set()
    used_ids = set()
    next_id = 1
    cleaned = []
    for raw in points:
        point = PointModel(**raw).normalised()
        if point['name'] in names:
            raise HTTPException(status_code=422, detail=f"点位名称重复: {point['name']}")
        names.add(point['name'])
        point_id = point['id']
        if point_id is None or point_id <= 0 or point_id in used_ids:
            while next_id in used_ids:
                next_id += 1
            point_id = next_id
        used_ids.add(point_id)
        point['id'] = point_id
        cleaned.append(point)
    return cleaned


def write_points(payload: PointsPayload) -> dict:
    cleaned = _validate_points([p.model_dump() for p in payload.points])
    data = {'frame': payload.frame or 'map', 'points': cleaned}
    points_file = _active_points_file()
    points_file.parent.mkdir(parents=True, exist_ok=True)
    if points_file.exists():
        backup = points_file.with_suffix('.yaml.bak')
        try:
            backup.write_bytes(points_file.read_bytes())
        except OSError:
            pass
    tmp = points_file.with_suffix('.yaml.tmp')
    with tmp.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
    tmp.replace(points_file)
    _mirror_legacy_points(data)
    return data


# ---------------------------------------------------------------------------
# 地图文本标注
# ---------------------------------------------------------------------------
COLOR_RE = re.compile(r'^#[0-9a-fA-F]{6}$')


class AnnotationModel(BaseModel):
    id: Optional[int] = None
    text: str = Field(min_length=1, max_length=200)
    x: float
    y: float
    yaw: float = 0.0
    color: str = '#2563eb'
    size: float = Field(default=14.0, ge=8.0, le=48.0)
    map: str = ''

    def normalised(self) -> dict:
        text = (self.text or '').strip()
        if not text:
            raise HTTPException(status_code=422, detail='标注文本不能为空')
        if len(text) > 200:
            raise HTTPException(status_code=422, detail='标注文本最长 200 个字符')
        color = (self.color or '#2563eb').strip()
        if not COLOR_RE.match(color):
            raise HTTPException(status_code=422, detail='颜色必须是 #RRGGBB')
        return {
            'id': self.id,
            'text': text,
            'x': _finite(self.x, 'x'),
            'y': _finite(self.y, 'y'),
            'yaw': _finite(self.yaw, 'yaw'),
            'color': color,
            'size': round(_finite(self.size, 'size'), 1),
            'map': (self.map or '').strip()[:256],
        }


class AnnotationsPayload(BaseModel):
    frame: str = 'map'
    annotations: list[AnnotationModel] = Field(default_factory=list)


def read_annotations() -> dict:
    annotations_file = _scoped_file('annotations.yaml', ANNOTATIONS_FILE)
    if not annotations_file.exists():
        return {'frame': 'map', 'annotations': []}
    with annotations_file.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    annotations = []
    for index, raw in enumerate(data.get('annotations') or [], start=1):
        try:
            annotations.append({
                'id': int(raw.get('id', index)),
                'text': str(raw['text']),
                'x': float(raw['x']),
                'y': float(raw['y']),
                'yaw': float(raw.get('yaw', 0.0)),
                'color': str(raw.get('color', '#2563eb')),
                'size': float(raw.get('size', 14.0)),
                'map': str(raw.get('map', '')),
            })
        except (KeyError, TypeError, ValueError):
            continue
    return {'frame': str(data.get('frame', 'map')), 'annotations': annotations}


def write_annotations(payload: AnnotationsPayload) -> dict:
    cleaned = []
    used_ids = set()
    next_id = 1
    for raw in payload.annotations:
        item = AnnotationModel(**raw.model_dump()).normalised()
        item_id = item['id']
        if item_id is None or item_id <= 0 or item_id in used_ids:
            while next_id in used_ids:
                next_id += 1
            item_id = next_id
        used_ids.add(item_id)
        item['id'] = item_id
        cleaned.append(item)
    data = {'frame': payload.frame or 'map', 'annotations': cleaned}
    annotations_file = _scoped_file('annotations.yaml', ANNOTATIONS_FILE)
    annotations_file.parent.mkdir(parents=True, exist_ok=True)
    if annotations_file.exists():
        backup = annotations_file.with_suffix('.yaml.bak')
        try:
            backup.write_bytes(annotations_file.read_bytes())
        except OSError:
            pass
    tmp = annotations_file.with_suffix('.yaml.tmp')
    with tmp.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
    tmp.replace(annotations_file)
    return data


# ---------------------------------------------------------------------------
# 任务存储
# ---------------------------------------------------------------------------
class MissionModel(BaseModel):
    name: str = Field(min_length=1, max_length=64)
    points: list[PointModel]
    loop: bool = False


class StartMissionRequest(BaseModel):
    mission_id: Optional[int] = None
    name: str = 'mission'
    points: Optional[list[PointModel]] = None
    loop: bool = False


class MissionStore:
    def __init__(self, path: Path) -> None:
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        with self._connect() as db:
            db.execute(
                '''CREATE TABLE IF NOT EXISTS missions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    points TEXT NOT NULL,
                    loop INTEGER NOT NULL DEFAULT 0,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )''')
            db.execute(
                '''CREATE TABLE IF NOT EXISTS mission_runs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    mission_id INTEGER,
                    name TEXT NOT NULL,
                    points TEXT NOT NULL,
                    loop INTEGER NOT NULL DEFAULT 0,
                    state TEXT NOT NULL,
                    message TEXT NOT NULL DEFAULT '',
                    total_points INTEGER NOT NULL DEFAULT 0,
                    completed_points INTEGER NOT NULL DEFAULT 0,
                    started_at TEXT,
                    finished_at TEXT NOT NULL,
                    duration_sec REAL
                )''')
            for table in ('missions', 'mission_runs'):
                columns = {row[1] for row in db.execute(f'PRAGMA table_info({table})')}
                if 'workspace_id' not in columns:
                    db.execute(f'ALTER TABLE {table} ADD COLUMN workspace_id INTEGER')
            db.commit()

    def backfill_workspace(self, workspace_id: int) -> None:
        """把旧版本没有工作区归属的任务/历史归到默认工作区。"""
        with self._lock, self._connect() as db:
            db.execute(
                'UPDATE missions SET workspace_id=? WHERE workspace_id IS NULL', (workspace_id,))
            db.execute(
                'UPDATE mission_runs SET workspace_id=? WHERE workspace_id IS NULL', (workspace_id,))
            db.commit()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=10)
        connection.row_factory = sqlite3.Row
        return connection

    @staticmethod
    def _row(row: sqlite3.Row) -> dict:
        return {
            'id': int(row['id']),
            'workspace_id': int(row['workspace_id']) if row['workspace_id'] is not None else None,
            'name': row['name'],
            'points': json.loads(row['points']),
            'loop': bool(row['loop']),
            'created_at': row['created_at'],
            'updated_at': row['updated_at'],
        }

    def list(self, workspace_id: Optional[int] = None) -> list[dict]:
        with self._lock, self._connect() as db:
            if workspace_id is None:
                rows = db.execute('SELECT * FROM missions ORDER BY id DESC').fetchall()
            else:
                rows = db.execute(
                    'SELECT * FROM missions WHERE workspace_id=? ORDER BY id DESC',
                    (workspace_id,)).fetchall()
        return [self._row(row) for row in rows]

    def get(self, mission_id: int, workspace_id: Optional[int] = None) -> Optional[dict]:
        with self._lock, self._connect() as db:
            if workspace_id is None:
                row = db.execute('SELECT * FROM missions WHERE id=?', (mission_id,)).fetchone()
            else:
                row = db.execute(
                    'SELECT * FROM missions WHERE id=? AND workspace_id=?',
                    (mission_id, workspace_id)).fetchone()
        return self._row(row) if row else None

    def create(self, name: str, points: list[dict], loop: bool,
               workspace_id: Optional[int] = None) -> dict:
        now = _now()
        with self._lock, self._connect() as db:
            cursor = db.execute(
                '''INSERT INTO missions(name, points, loop, workspace_id, created_at, updated_at)
                   VALUES (?,?,?,?,?,?)''',
                (name, json.dumps(points, ensure_ascii=False), int(loop), workspace_id, now, now))
            db.commit()
            mission_id = int(cursor.lastrowid)
        return self.get(mission_id) or {}

    def update(self, mission_id: int, name: str, points: list[dict], loop: bool,
               workspace_id: Optional[int] = None) -> Optional[dict]:
        with self._lock, self._connect() as db:
            if workspace_id is None:
                cursor = db.execute(
                    'UPDATE missions SET name=?, points=?, loop=?, updated_at=? WHERE id=?',
                    (name, json.dumps(points, ensure_ascii=False), int(loop), _now(), mission_id))
            else:
                cursor = db.execute(
                    '''UPDATE missions SET name=?, points=?, loop=?, updated_at=?
                       WHERE id=? AND workspace_id=?''',
                    (name, json.dumps(points, ensure_ascii=False), int(loop), _now(),
                     mission_id, workspace_id))
            db.commit()
            if cursor.rowcount == 0:
                return None
        return self.get(mission_id, workspace_id)

    def delete(self, mission_id: int, workspace_id: Optional[int] = None) -> bool:
        with self._lock, self._connect() as db:
            if workspace_id is None:
                cursor = db.execute('DELETE FROM missions WHERE id=?', (mission_id,))
            else:
                cursor = db.execute(
                    'DELETE FROM missions WHERE id=? AND workspace_id=?',
                    (mission_id, workspace_id))
            db.commit()
            return cursor.rowcount > 0

    # ------------------------------------------------------------ run history
    @staticmethod
    def _run_row(row: sqlite3.Row) -> dict:
        return {
            'id': int(row['id']),
            'workspace_id': int(row['workspace_id']) if row['workspace_id'] is not None else None,
            'mission_id': int(row['mission_id']) if row['mission_id'] is not None else None,
            'name': row['name'],
            'points': json.loads(row['points']),
            'loop': bool(row['loop']),
            'state': row['state'],
            'message': row['message'],
            'total_points': int(row['total_points']),
            'completed_points': int(row['completed_points']),
            'started_at': row['started_at'],
            'finished_at': row['finished_at'],
            'duration_sec': row['duration_sec'],
        }

    def add_run(self, state: dict) -> dict:
        points = state.get('points') or []
        started_at = state.get('started_at')
        finished_at = state.get('finished_at') or _now()
        duration = None
        if started_at:
            try:
                begin = datetime.fromisoformat(started_at)
                end = datetime.fromisoformat(finished_at)
                duration = round(max(0.0, (end - begin).total_seconds()), 1)
            except (TypeError, ValueError):
                duration = None
        with self._lock, self._connect() as db:
            cursor = db.execute(
                '''INSERT INTO mission_runs(
                    workspace_id, mission_id, name, points, loop, state, message,
                    total_points, completed_points, started_at, finished_at, duration_sec)
                   VALUES (?,?,?,?,?,?,?,?,?,?,?,?)''',
                (
                    state.get('workspace_id'),
                    state.get('mission_id'),
                    state.get('name') or '未命名任务',
                    json.dumps(points, ensure_ascii=False),
                    int(bool(state.get('loop'))),
                    state.get('state') or 'unknown',
                    state.get('message') or '',
                    len(points),
                    min(int(state.get('index') or 0), len(points)),
                    started_at,
                    finished_at,
                    duration,
                ))
            db.commit()
            run_id = int(cursor.lastrowid)
            row = db.execute('SELECT * FROM mission_runs WHERE id=?', (run_id,)).fetchone()
        return self._run_row(row) if row else {}

    def list_runs(self, limit: int = 50, workspace_id: Optional[int] = None) -> list[dict]:
        with self._lock, self._connect() as db:
            if workspace_id is None:
                rows = db.execute(
                    'SELECT * FROM mission_runs ORDER BY id DESC LIMIT ?', (limit,)).fetchall()
            else:
                rows = db.execute(
                    'SELECT * FROM mission_runs WHERE workspace_id=? ORDER BY id DESC LIMIT ?',
                    (workspace_id, limit)).fetchall()
        return [self._run_row(row) for row in rows]

    def update_run(self, run_id: int, state: str, message: str,
                   completed_points: int) -> Optional[dict]:
        """任务被恢复后再次结束时，更新同一条历史而不是新增一条。"""
        finished_at = _now()
        with self._lock, self._connect() as db:
            row = db.execute(
                'SELECT started_at FROM mission_runs WHERE id=?', (run_id,)).fetchone()
            if row is None:
                return None
            duration = None
            if row['started_at']:
                try:
                    begin = datetime.fromisoformat(row['started_at'])
                    end = datetime.fromisoformat(finished_at)
                    duration = round(max(0.0, (end - begin).total_seconds()), 1)
                except (TypeError, ValueError):
                    duration = None
            db.execute(
                '''UPDATE mission_runs SET state=?, message=?, completed_points=?,
                   finished_at=?, duration_sec=? WHERE id=?''',
                (state, message, completed_points, finished_at, duration, run_id))
            db.commit()
            updated = db.execute('SELECT * FROM mission_runs WHERE id=?', (run_id,)).fetchone()
        return self._run_row(updated) if updated else None

    def clear_runs(self, workspace_id: Optional[int] = None) -> int:
        with self._lock, self._connect() as db:
            if workspace_id is None:
                cursor = db.execute('DELETE FROM mission_runs')
            else:
                cursor = db.execute(
                    'DELETE FROM mission_runs WHERE workspace_id=?', (workspace_id,))
            db.commit()
            return cursor.rowcount


# ---------------------------------------------------------------------------
# 工作区（地图 + 巡检点位 + 标注的隔离单元）
# ---------------------------------------------------------------------------
class WorkspaceStore:
    """每个工作区有自己的 points.yaml / annotations.yaml / maps/prior.pcd。

    默认工作区沿用 src/me_nav2_bringup/config 下的全局文件，保持旧用法兼容；
    新建工作区的数据放在 web_backend_data/workspaces/ws_<id>_<slug>/ 下。
    """

    def __init__(self, path: Path) -> None:
        self.path = path
        self._lock = threading.RLock()
        path.parent.mkdir(parents=True, exist_ok=True)
        WORKSPACES_DIR.mkdir(parents=True, exist_ok=True)
        with self._connect() as db:
            db.execute(
                '''CREATE TABLE IF NOT EXISTS workspaces (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL UNIQUE,
                    slug TEXT NOT NULL,
                    map_yaml TEXT,
                    map_pgm TEXT,
                    prior_pcd TEXT,
                    is_default INTEGER NOT NULL DEFAULT 0,
                    active_plan_id INTEGER,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )''')
            columns = {row[1] for row in db.execute('PRAGMA table_info(workspaces)')}
            if 'active_plan_id' not in columns:
                db.execute('ALTER TABLE workspaces ADD COLUMN active_plan_id INTEGER')
            db.execute(
                '''CREATE TABLE IF NOT EXISTS plans (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    workspace_id INTEGER NOT NULL,
                    name TEXT NOT NULL,
                    slug TEXT NOT NULL,
                    is_default INTEGER NOT NULL DEFAULT 0,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    UNIQUE(workspace_id, name)
                )''')
            db.commit()
        self._ensure_default()
        self._ensure_plans()
        self._ensure_points()
        self._ensure_annotations()
        self._ensure_workspace_maps()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=10)
        connection.row_factory = sqlite3.Row
        return connection

    @staticmethod
    def _row(row: sqlite3.Row) -> dict:
        directory = WORKSPACES_DIR / f"ws_{int(row['id'])}_{row['slug']}"
        return {
            'id': int(row['id']),
            'name': row['name'],
            'slug': row['slug'],
            'map_yaml': row['map_yaml'],
            'map_pgm': row['map_pgm'],
            'prior_pcd': row['prior_pcd'],
            'is_default': bool(row['is_default']),
            'active_plan_id': (int(row['active_plan_id'])
                               if row['active_plan_id'] is not None else None),
            'directory': str(directory),
            'created_at': row['created_at'],
            'updated_at': row['updated_at'],
        }

    def list(self) -> list[dict]:
        with self._lock, self._connect() as db:
            rows = db.execute('SELECT * FROM workspaces ORDER BY is_default DESC, id ASC').fetchall()
        return [self._row(row) for row in rows]

    def get(self, workspace_id: int) -> Optional[dict]:
        with self._lock, self._connect() as db:
            row = db.execute('SELECT * FROM workspaces WHERE id=?', (workspace_id,)).fetchone()
        return self._row(row) if row else None

    def _read_active_id(self) -> Optional[int]:
        try:
            data = json.loads(ACTIVE_WORKSPACE_FILE.read_text(encoding='utf-8'))
            return int(data.get('id'))
        except (OSError, ValueError, TypeError, AttributeError):
            return None

    def active(self) -> Optional[dict]:
        with self._lock:
            workspace_id = self._read_active_id()
            workspace = self.get(workspace_id) if workspace_id else None
            if workspace is None:
                with self._connect() as db:
                    row = db.execute(
                        'SELECT id FROM workspaces ORDER BY is_default DESC, id ASC LIMIT 1'
                    ).fetchone()
                workspace = self.get(int(row['id'])) if row else None
                if workspace is not None:
                    self._write_active_id(workspace['id'])
            return workspace

    def _write_active_id(self, workspace_id: int) -> None:
        ACTIVE_WORKSPACE_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = ACTIVE_WORKSPACE_FILE.with_suffix('.tmp')
        tmp.write_text(json.dumps({'id': int(workspace_id)}), encoding='utf-8')
        tmp.replace(ACTIVE_WORKSPACE_FILE)

    def set_active(self, workspace_id: int) -> Optional[dict]:
        workspace = self.get(workspace_id)
        if workspace is None:
            return None
        with self._lock:
            self._write_active_id(workspace_id)
        return workspace

    def create(self, name: str) -> dict:
        name = name.strip()
        if not name or len(name) > 64:
            raise HTTPException(status_code=422, detail='工作区名称不能为空且最长 64 个字符')
        now = _now()
        slug = re.sub(r'[^a-z0-9]+', '-', name.lower()).strip('-')[:32] or 'workspace'
        with self._lock, self._connect() as db:
            try:
                cursor = db.execute(
                    '''INSERT INTO workspaces(name, slug, is_default, created_at, updated_at)
                       VALUES (?,?,0,?,?)''', (name, slug, now, now))
            except sqlite3.IntegrityError:
                raise HTTPException(status_code=409, detail=f'工作区名称已存在: {name}')
            db.commit()
            workspace_id = int(cursor.lastrowid)
        workspace = self.get(workspace_id)
        if workspace:
            Path(workspace['directory']).mkdir(parents=True, exist_ok=True)
            now = _now()
            with self._lock, self._connect() as db:
                cursor = db.execute(
                    '''INSERT INTO plans(workspace_id, name, slug, is_default,
                       created_at, updated_at) VALUES (?,?,?,1,?,?)''',
                    (workspace_id, '默认巡检', 'default', now, now))
                db.commit()
                plan_id = int(cursor.lastrowid)
            self.set_active_plan(workspace_id, plan_id)
            workspace = self.get(workspace_id)
        return workspace or {}

    def update(self, workspace_id: int, **fields: Any) -> Optional[dict]:
        allowed = ('map_yaml', 'map_pgm', 'prior_pcd')
        updates = {key: value for key, value in fields.items() if key in allowed}
        if not updates:
            return self.get(workspace_id)
        updates['updated_at'] = _now()
        assignments = ', '.join(f'{key}=?' for key in updates)
        values = list(updates.values()) + [workspace_id]
        with self._lock, self._connect() as db:
            cursor = db.execute(
                f'UPDATE workspaces SET {assignments} WHERE id=?', values)
            db.commit()
            if cursor.rowcount == 0:
                return None
        return self.get(workspace_id)

    def delete(self, workspace_id: int) -> bool:
        workspace = self.get(workspace_id)
        if workspace is None:
            return False
        if workspace['is_default']:
            raise HTTPException(status_code=409, detail='默认工作区不能删除')
        active = self.active()
        if active is not None and active['id'] == workspace_id:
            raise HTTPException(status_code=409, detail='请先切换到其他工作区再删除')
        with self._lock, self._connect() as db:
            db.execute('DELETE FROM missions WHERE workspace_id=?', (workspace_id,))
            db.execute('DELETE FROM mission_runs WHERE workspace_id=?', (workspace_id,))
            db.execute('DELETE FROM plans WHERE workspace_id=?', (workspace_id,))
            db.execute('DELETE FROM workspaces WHERE id=?', (workspace_id,))
            db.commit()
        shutil.rmtree(workspace['directory'], ignore_errors=True)
        return True

    # ------------------------------------------------------------ plans
    @staticmethod
    def _plan_row(row: sqlite3.Row, workspace: dict) -> dict:
        plan_id = int(row['id'])
        slug = row['slug']
        if row['is_default']:
            points_file = Path(workspace['directory']) / 'points.yaml'
        else:
            points_file = Path(workspace['directory']) / 'plans' / \
                f'plan_{plan_id}_{slug}' / 'points.yaml'
        return {
            'id': plan_id,
            'workspace_id': int(row['workspace_id']),
            'name': row['name'],
            'slug': slug,
            'is_default': bool(row['is_default']),
            'points_file': str(points_file),
            'directory': str(points_file.parent),
            'created_at': row['created_at'],
            'updated_at': row['updated_at'],
        }

    def plans(self, workspace_id: int) -> list[dict]:
        workspace = self.get(workspace_id)
        if workspace is None:
            return []
        with self._lock, self._connect() as db:
            rows = db.execute(
                'SELECT * FROM plans WHERE workspace_id=? ORDER BY is_default DESC, id ASC',
                (workspace_id,)).fetchall()
        return [self._plan_row(row, workspace) for row in rows]

    def plan(self, plan_id: int) -> Optional[dict]:
        with self._lock, self._connect() as db:
            row = db.execute('SELECT * FROM plans WHERE id=?', (plan_id,)).fetchone()
        if row is None:
            return None
        workspace = self.get(int(row['workspace_id']))
        if workspace is None:
            return None
        return self._plan_row(row, workspace)

    def active_plan(self, workspace_id: int) -> Optional[dict]:
        workspace = self.get(workspace_id)
        if workspace is None:
            return None
        plans = self.plans(workspace_id)
        if not plans:
            return None
        active_id = workspace.get('active_plan_id')
        for plan in plans:
            if plan['id'] == active_id:
                return plan
        default = next((plan for plan in plans if plan['is_default']), plans[0])
        self.set_active_plan(workspace_id, default['id'])
        return default

    def set_active_plan(self, workspace_id: int, plan_id: int) -> bool:
        plan = self.plan(plan_id)
        if plan is None or plan['workspace_id'] != workspace_id:
            return False
        with self._lock, self._connect() as db:
            db.execute(
                'UPDATE workspaces SET active_plan_id=?, updated_at=? WHERE id=?',
                (plan_id, _now(), workspace_id))
            db.commit()
        return True

    def create_plan(self, workspace_id: int, name: str) -> dict:
        workspace = self.get(workspace_id)
        if workspace is None:
            raise HTTPException(status_code=404, detail='工作区不存在')
        name = (name or '').strip()
        if not name or len(name) > 64:
            raise HTTPException(status_code=422, detail='方案名称不能为空且最长 64 个字符')
        slug = re.sub(r'[^a-z0-9]+', '-', name.lower()).strip('-')[:32] or 'plan'
        now = _now()
        with self._lock, self._connect() as db:
            try:
                cursor = db.execute(
                    '''INSERT INTO plans(workspace_id, name, slug, is_default,
                       created_at, updated_at) VALUES (?,?,?,0,?,?)''',
                    (workspace_id, name, slug, now, now))
            except sqlite3.IntegrityError:
                raise HTTPException(status_code=409, detail=f'这个工作区已有同名方案: {name}')
            db.commit()
            plan_id = int(cursor.lastrowid)
        plan = self.plan(plan_id)
        if plan:
            Path(plan['directory']).mkdir(parents=True, exist_ok=True)
        return plan or {}

    def rename_plan(self, plan_id: int, name: str) -> Optional[dict]:
        plan = self.plan(plan_id)
        if plan is None:
            return None
        name = (name or '').strip()
        if not name or len(name) > 64:
            raise HTTPException(status_code=422, detail='方案名称不能为空且最长 64 个字符')
        with self._lock, self._connect() as db:
            try:
                db.execute(
                    'UPDATE plans SET name=?, updated_at=? WHERE id=?',
                    (name, _now(), plan_id))
            except sqlite3.IntegrityError:
                raise HTTPException(status_code=409, detail=f'这个工作区已有同名方案: {name}')
            db.commit()
        return self.plan(plan_id)

    def delete_plan(self, plan_id: int) -> bool:
        plan = self.plan(plan_id)
        if plan is None:
            return False
        if plan['is_default']:
            raise HTTPException(status_code=409, detail='默认方案不能删除')
        active = self.active_plan(plan['workspace_id'])
        if active is not None and active['id'] == plan_id:
            raise HTTPException(status_code=409, detail='请先切换到其他方案再删除')
        with self._lock, self._connect() as db:
            db.execute('DELETE FROM plans WHERE id=?', (plan_id,))
            db.commit()
        if not plan['is_default']:
            shutil.rmtree(plan['directory'], ignore_errors=True)
        return True

    def _ensure_plans(self) -> None:
        for workspace in self.list():
            plans = self.plans(workspace['id'])
            if not plans:
                now = _now()
                with self._connect() as db:
                    cursor = db.execute(
                        '''INSERT INTO plans(workspace_id, name, slug, is_default,
                           created_at, updated_at) VALUES (?,?,?,1,?,?)''',
                        (workspace['id'], '默认巡检', 'default', now, now))
                    db.commit()
                    plan_id = int(cursor.lastrowid)
                self.set_active_plan(workspace['id'], plan_id)
            elif not any(plan['id'] == workspace.get('active_plan_id') for plan in plans):
                default = next((plan for plan in plans if plan['is_default']), plans[0])
                self.set_active_plan(workspace['id'], default['id'])

    def _ensure_points(self) -> None:
        """默认工作区的默认方案点位表迁移到工作区目录。

        旧的 config/waypoints.yaml 仍会被写一份镜像，供 voice_agent /
        waypoint_recorder 等仍读取全局路径的节点使用。
        """
        for workspace in self.list():
            plans = self.plans(workspace['id'])
            default_plan = next((plan for plan in plans if plan['is_default']), None)
            if default_plan is None:
                continue
            target = Path(default_plan['points_file'])
            if not workspace.get('is_default') or not POINTS_FILE.exists():
                continue
            try:
                target.parent.mkdir(parents=True, exist_ok=True)
                if not target.exists():
                    target.write_bytes(POINTS_FILE.read_bytes())
                elif (POINTS_FILE.stat().st_mtime > target.stat().st_mtime and
                      POINTS_FILE.read_bytes() != target.read_bytes()):
                    # 外部工具（waypoint_recorder 等）更新了全局文件时才同步
                    target.write_bytes(POINTS_FILE.read_bytes())
            except OSError:
                pass

    def _ensure_workspace_maps(self) -> None:
        """把指向工作区外的二维地图和先验 PCD 复制进工作区目录。"""
        active = self.active()
        for workspace in self.list():
            updates = {}
            workspace_dir = Path(workspace['directory']).resolve()
            map_yaml = workspace.get('map_yaml')
            if map_yaml:
                source = (WORKSPACE / map_yaml).resolve()
                try:
                    source.relative_to(workspace_dir)
                except ValueError:
                    updates.update(self._absorbe_map(workspace_dir, source))
            prior_pcd = workspace.get('prior_pcd')
            if prior_pcd:
                source = (WORKSPACE / prior_pcd).resolve()
                try:
                    source.relative_to(workspace_dir)
                except ValueError:
                    target_dir = workspace_dir / 'prior_pcd'
                    try:
                        target_dir.mkdir(parents=True, exist_ok=True)
                        target = target_dir / source.name
                        shutil.copy2(source, target)
                        updates['prior_pcd'] = _rel(target)
                    except OSError:
                        pass
            if not updates:
                continue
            updated = self.update(workspace['id'], **updates)
            if updated and active is not None and active['id'] == workspace['id']:
                _apply_workspace_map(updated)

    @staticmethod
    def _absorbe_map(workspace_dir: Path, yaml_path: Path) -> dict:
        try:
            with yaml_path.open(encoding='utf-8') as stream:
                meta = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError):
            return {}
        image = str(meta.get('image') or '')
        if not image:
            return {}
        image_path = (yaml_path.parent / image).resolve()
        if not image_path.exists():
            return {}
        target_dir = workspace_dir / 'maps' / 'migrated'
        try:
            target_dir.mkdir(parents=True, exist_ok=True)
            image_target = target_dir / image_path.name
            yaml_target = target_dir / yaml_path.name
            shutil.copy2(image_path, image_target)
            if Path(image).name == image:
                shutil.copy2(yaml_path, yaml_target)
            else:
                meta['image'] = image_target.name
                with yaml_target.open('w', encoding='utf-8') as stream:
                    yaml.safe_dump(meta, stream, allow_unicode=True, sort_keys=False)
        except OSError:
            return {}
        return {
            'map_yaml': _rel(yaml_target),
            'map_pgm': _rel(image_target),
        }

    def _ensure_annotations(self) -> None:
        """把默认工作区原来的全局标注文件迁移到工作区目录。

        迁移后所有读写都走 <workspace>/annotations.yaml；旧全局文件保留不删，
        作为迁移前的一份备份。
        """
        for workspace in self.list():
            target = Path(workspace['directory']) / 'annotations.yaml'
            if target.exists():
                continue
            if workspace.get('is_default') and ANNOTATIONS_FILE.exists():
                try:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(ANNOTATIONS_FILE.read_bytes())
                except OSError:
                    pass

    def _ensure_default(self) -> None:
        with self._connect() as db:
            count = int(db.execute('SELECT COUNT(*) FROM workspaces').fetchone()[0])
        if count:
            if self._read_active_id() is None:
                first = self.list()[0] if self.list() else None
                if first:
                    self._write_active_id(first['id'])
            return
        now = _now()
        map_yaml, map_pgm, prior_pcd = self._legacy_map_fields()
        with self._connect() as db:
            cursor = db.execute(
                '''INSERT INTO workspaces(name, slug, map_yaml, map_pgm, prior_pcd,
                   is_default, created_at, updated_at) VALUES (?,?,?,?,?,1,?,?)''',
                ('默认工作区', 'default', map_yaml, map_pgm, prior_pcd, now, now))
            db.commit()
            workspace_id = int(cursor.lastrowid)
        self._write_active_id(workspace_id)
        workspace = self.get(workspace_id)
        if workspace:
            Path(workspace['directory']).mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _legacy_map_fields() -> tuple[Optional[str], Optional[str], Optional[str]]:
        if not MAP_CONFIG_FILE.exists():
            return None, None, None
        try:
            with MAP_CONFIG_FILE.open(encoding='utf-8') as stream:
                cfg = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError):
            return None, None, None
        pkg_root = WORKSPACE / 'src' / 'me_nav2_bringup'

        def resolve(value: Optional[str]) -> Optional[Path]:
            if not value:
                return None
            path = Path(str(value))
            if not path.is_absolute():
                path = pkg_root / path
            return path.resolve()

        yaml_path = resolve(cfg.get('map_yaml_file'))
        pcd_path = resolve(cfg.get('prior_pcd_file'))
        pgm_path = None
        if yaml_path is not None and yaml_path.exists():
            try:
                with yaml_path.open(encoding='utf-8') as stream:
                    meta = yaml.safe_load(stream) or {}
                image = meta.get('image')
                if image:
                    pgm_path = (yaml_path.parent / str(image)).resolve()
            except (OSError, yaml.YAMLError):
                pass
        return (
            _rel(yaml_path) if yaml_path and yaml_path.exists() else None,
            _rel(pgm_path) if pgm_path and pgm_path.exists() else None,
            _rel(pcd_path) if pcd_path and pcd_path.exists() else None,
        )


def _apply_workspace_map(workspace: dict) -> list[str]:
    """把工作区地图写入 map_config.yaml，导航栈下次启动时生效。

    只替换对应的两行，保留文件里的注释和排版。
    """
    warnings = []
    if not MAP_CONFIG_FILE.exists():
        warnings.append('找不到 map_config.yaml，未写入导航配置')
        return warnings
    try:
        text = MAP_CONFIG_FILE.read_text(encoding='utf-8')
    except OSError as exc:
        warnings.append(f'读取 map_config.yaml 失败: {exc}')
        return warnings
    updates = []
    if workspace.get('map_yaml'):
        updates.append(('map_yaml_file', str((WORKSPACE / workspace['map_yaml']).resolve())))
    else:
        warnings.append('这个工作区还没有二维地图，map_yaml_file 未改动')
    if workspace.get('prior_pcd'):
        updates.append(('prior_pcd_file', str((WORKSPACE / workspace['prior_pcd']).resolve())))
    else:
        warnings.append('这个工作区还没有先验 PCD，prior_pcd_file 未改动；追踪模式需要先上传 PCD')
    if not updates:
        return warnings
    for key, value in updates:
        pattern = re.compile(rf'^{re.escape(key)}\s*:.*$', re.MULTILINE)
        replacement = f'{key}: "{value}"'
        if pattern.search(text):
            text = pattern.sub(replacement, text, count=1)
        else:
            text = text.rstrip() + '\n' + replacement + '\n'
    try:
        backup = MAP_CONFIG_FILE.with_suffix('.yaml.bak')
        backup.write_bytes(MAP_CONFIG_FILE.read_bytes())
        tmp = MAP_CONFIG_FILE.with_suffix('.yaml.tmp')
        tmp.write_text(text, encoding='utf-8')
        tmp.replace(MAP_CONFIG_FILE)
    except OSError as exc:
        warnings.append(f'写入 map_config.yaml 失败: {exc}')
    return warnings


# ---------------------------------------------------------------------------
# ROS / 任务执行
# ---------------------------------------------------------------------------
ROS_AVAILABLE = False
ROS_ERROR = ''
try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    ROS_AVAILABLE = True
except Exception as exc:  # pragma: no cover - depends on environment
    ROS_ERROR = f'{type(exc).__name__}: {exc}'


if ROS_AVAILABLE:
    class BackendNode(Node):
        def __init__(self) -> None:
            super().__init__('web_backend')
            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.status: Optional[dict] = None
            self.status_time: Optional[float] = None
            self.create_subscription(String, '/robot/status', self._on_status, latched)
            self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        def _on_status(self, msg: String) -> None:
            try:
                self.status = json.loads(msg.data)
                self.status_time = time.time()
            except (ValueError, TypeError):
                pass


    class MissionExecutor:
        """服务端任务执行器。

        NavigateToPose action 的结果/反馈回调在 ROS executor 线程里执行；
        FastAPI 线程只修改状态并调用 cancel。所有可变状态都由 self._lock 保护。
        """

        def __init__(self, node: BackendNode) -> None:
            self.node = node
            self._lock = threading.RLock()
            self._generation = 0
            self._active = False
            self._goal_handle = None
            self._pending_at = 0.0
            self._state: dict[str, Any] = {
                'state': 'idle',
                'mission_id': None,
                'name': '',
                'points': [],
                'index': 0,
                'loop': False,
                'distance_remaining': None,
                'message': '',
                'started_at': None,
                'updated_at': _now(),
                'run_id': None,
                'workspace_id': None,
            }
            saved = _load_runtime()
            if saved:
                self._state.update(saved)
                if self._state.get('workspace_id') is None and workspace_store is not None:
                    active = workspace_store.active()
                    if active is not None:
                        self._state['workspace_id'] = active['id']
                if self._state.get('state') in ('running', 'paused'):
                    self._state['state'] = 'interrupted'
                    self._state['message'] = '后端重启，任务已中断；可点击“继续任务”'
                    self._state['updated_at'] = _now()
                elif self._state.get('state') == 'interrupted':
                    self._state['message'] = self._state.get('message') or '任务已中断；可点击“继续任务”'
                self._state['distance_remaining'] = None
                _persist_runtime(self._state)
                # 上次进程退出时没来得及落库的运行记录，补一条 interrupted
                self._record_run()
            node.create_timer(0.1, self._tick)

        # ------------------------------------------------------------ helpers
        def _set_state(self, state: str, message: str = '') -> None:
            self._state['state'] = state
            self._state['message'] = message
            self._state['updated_at'] = _now()
            _persist_runtime(self._state)
            if state in ('succeeded', 'failed', 'cancelled', 'interrupted'):
                self._record_run()

        def _record_run(self) -> None:
            """把终态写入历史；同一任务恢复后再次结束会更新原记录。"""
            with self._lock:
                if not self._state.get('started_at') or not self._state.get('points'):
                    return
                if self._state.get('state') not in (
                        'succeeded', 'failed', 'cancelled', 'interrupted'):
                    return
                snapshot = dict(self._state)
                snapshot['points'] = [dict(p) for p in self._state['points']]
                run_id = snapshot.get('run_id')
                snapshot['finished_at'] = _now()
                if snapshot.get('workspace_id') is None and workspace_store is not None:
                    active = workspace_store.active()
                    if active is not None:
                        snapshot['workspace_id'] = active['id']
            try:
                if run_id:
                    store.update_run(
                        int(run_id), snapshot['state'], snapshot.get('message') or '',
                        min(int(snapshot.get('index') or 0), len(snapshot['points'])))
                else:
                    row = store.add_run(snapshot)
                    with self._lock:
                        self._state['run_id'] = row.get('id')
                        _persist_runtime(self._state)
            except Exception:
                pass

        def snapshot(self) -> dict:
            with self._lock:
                data = dict(self._state)
                data['points'] = [dict(p) for p in self._state['points']]
                return data

        def _cancel_current(self) -> None:
            handle = self._goal_handle
            self._goal_handle = None
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass

        # ------------------------------------------------------------ public
        def start(self, name: str, points: list[dict], loop: bool,
                  mission_id: Optional[int] = None,
                  workspace_id: Optional[int] = None) -> dict:
            if not points:
                raise HTTPException(status_code=422, detail='任务至少需要一个点位')
            with self._lock:
                self._generation += 1
                self._active = False
                self._pending_at = 0.0
                self._state.update({
                    'state': 'running',
                    'mission_id': mission_id,
                    'workspace_id': workspace_id,
                    'name': name,
                    'points': [dict(p) for p in points],
                    'index': 0,
                    'loop': bool(loop),
                    'distance_remaining': None,
                    'message': '',
                    'started_at': _now(),
                    'updated_at': _now(),
                    'run_id': None,
                })
                _persist_runtime(self._state)
                old_handle = self._goal_handle
                self._goal_handle = None
            if old_handle is not None:
                try:
                    old_handle.cancel_goal_async()
                except Exception:
                    pass
            return self.snapshot()

        def pause(self) -> dict:
            with self._lock:
                if self._state['state'] != 'running':
                    return self.snapshot()
                self._set_state('paused', '任务已暂停')
                self._generation += 1
                self._active = False
            self._cancel_current()
            return self.snapshot()

        def resume(self) -> dict:
            with self._lock:
                if self._state['state'] not in ('paused', 'interrupted'):
                    return self.snapshot()
                if not self._state.get('points'):
                    self._set_state('failed', '没有可继续的点位')
                    return self.snapshot()
                if self._state['index'] >= len(self._state['points']):
                    self._state['index'] = 0
                self._generation += 1
                self._active = False
                self._pending_at = 0.0
                self._set_state('running', '任务已继续')
            return self.snapshot()

        def cancel(self) -> dict:
            with self._lock:
                self._generation += 1
                self._active = False
                self._set_state('cancelled', '任务已取消')
            self._cancel_current()
            return self.snapshot()

        def reset(self) -> dict:
            with self._lock:
                self._generation += 1
                self._active = False
                self._pending_at = 0.0
                handle = self._goal_handle
                self._goal_handle = None
                self._state = {
                    'state': 'idle', 'mission_id': None, 'name': '',
                    'points': [], 'index': 0, 'loop': False,
                    'distance_remaining': None, 'message': '',
                    'started_at': None, 'updated_at': _now(), 'run_id': None,
                    'workspace_id': None,
                }
                _persist_runtime(self._state)
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            return self.snapshot()

        # ------------------------------------------------------------ tick
        def _tick(self) -> None:
            with self._lock:
                if self._state['state'] != 'running' or self._active:
                    return
                if time.monotonic() < self._pending_at:
                    return
                points = self._state['points']
                if not points:
                    self._set_state('succeeded', '任务为空')
                    return
                if self._state['index'] >= len(points):
                    if self._state['loop']:
                        self._state['index'] = 0
                    else:
                        self._set_state('succeeded', '全部点位已到达')
                        return
                generation = self._generation
                index = self._state['index']
            self._send_current(generation, index)

        def _send_current(self, generation: int, index: int) -> None:
            with self._lock:
                if generation != self._generation or self._state['state'] != 'running':
                    return
                points = self._state['points']
                if index >= len(points):
                    return
                point = dict(points[index])
            goal = NavigateToPose.Goal()
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = float(point['x'])
            pose.pose.position.y = float(point['y'])
            yaw = float(point.get('yaw', 0.0))
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal.pose = pose

            try:
                available = self.node.action_client.wait_for_server(timeout_sec=1.0)
            except Exception:
                available = False
            if not available:
                with self._lock:
                    if generation == self._generation:
                        self._active = False
                        self._set_state('failed', 'navigate_to_pose action server 不可用')
                return

            with self._lock:
                if generation != self._generation or self._state['state'] != 'running':
                    return
                self._active = True
                self._state['distance_remaining'] = None

            future = self.node.action_client.send_goal_async(
                goal, feedback_callback=self._feedback_cb)
            future.add_done_callback(
                lambda done, gen=generation: self._goal_response_cb(done, gen))

        def _goal_response_cb(self, future, generation: int) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:
                with self._lock:
                    if generation == self._generation:
                        self._active = False
                        self._set_state('failed', f'发送目标失败: {exc}')
                return
            with self._lock:
                if generation != self._generation:
                    if goal_handle is not None and goal_handle.accepted:
                        try:
                            goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                    return
                if goal_handle is None or not goal_handle.accepted:
                    self._active = False
                    self._set_state('failed', '导航目标被拒绝')
                    return
                self._goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done, gen=generation: self._result_cb(done, gen))

        def _feedback_cb(self, msg) -> None:
            try:
                distance = msg.feedback.distance_remaining
            except Exception:
                return
            with self._lock:
                self._state['distance_remaining'] = round(float(distance), 2)

        def _result_cb(self, future, generation: int) -> None:
            try:
                result = future.result()
                status = int(result.status)
            except Exception as exc:
                status = -1
                error = str(exc)
            with self._lock:
                if generation != self._generation:
                    return
                self._active = False
                self._goal_handle = None
                if self._state['state'] in ('paused', 'cancelled'):
                    return
                if status == GoalStatus.STATUS_SUCCEEDED:
                    completed = self._state['index']
                    point = (self._state['points'][completed]
                             if completed < len(self._state['points']) else {})
                    try:
                        dwell = max(0.0, float(point.get('dwell') or 0.0))
                    except (TypeError, ValueError):
                        dwell = 0.0
                    self._state['index'] += 1
                    if self._state['index'] >= len(self._state['points']) and not self._state['loop']:
                        self._set_state('succeeded', '全部点位已到达')
                    else:
                        self._pending_at = time.monotonic() + 1.0 + dwell
                        name = point.get('name') or f'point_{completed + 1}'
                        self._state['message'] = (
                            f'到达 {name}，停留 {dwell:g}s' if dwell > 0 else '前往下一个点位')
                        self._state['updated_at'] = _now()
                        _persist_runtime(self._state)
                elif status == GoalStatus.STATUS_CANCELED:
                    self._set_state('cancelled', '导航目标被取消')
                else:
                    detail = error if status == -1 else f'status={status}'
                    self._set_state('failed', f'导航失败: {detail}')


# ---------------------------------------------------------------------------
# 全局对象
# ---------------------------------------------------------------------------
store = MissionStore(DB_PATH)
workspace_store = WorkspaceStore(DB_PATH)
_default_workspace = next(
    (item for item in workspace_store.list() if item.get('is_default')), None)
if _default_workspace is not None:
    store.backfill_workspace(_default_workspace['id'])
ros_node: Optional['BackendNode'] = None
mission_executor: Optional['MissionExecutor'] = None
ros_executor = None
ros_thread: Optional[threading.Thread] = None


def ros_ready() -> bool:
    return ROS_AVAILABLE and ros_node is not None and mission_executor is not None


def _active_workspace_id() -> Optional[int]:
    if workspace_store is None:
        return None
    workspace = workspace_store.active()
    return workspace['id'] if workspace else None


def start_ros() -> None:
    global ros_node, mission_executor, ros_executor, ros_thread
    if not ROS_AVAILABLE:
        return
    if ros_ready():
        return
    rclpy.init(args=None)
    ros_node = BackendNode()
    mission_executor = MissionExecutor(ros_node)
    ros_executor = MultiThreadedExecutor()
    ros_executor.add_node(ros_node)
    ros_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    ros_thread.start()


def stop_ros() -> None:
    global ros_node, mission_executor, ros_executor
    if ros_executor is not None:
        try:
            ros_executor.shutdown()
        except Exception:
            pass
    if ros_node is not None:
        try:
            ros_node.destroy_node()
        except Exception:
            pass
    if ROS_AVAILABLE:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    ros_node = None
    mission_executor = None
    ros_executor = None


# ---------------------------------------------------------------------------
# FastAPI
# ---------------------------------------------------------------------------
app = FastAPI(title='BUNKER Navigation Workbench API', version='0.1.0')
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_methods=['*'],
    allow_headers=['*'],
)


@app.middleware('http')
async def _auth_middleware(request: Request, call_next):
    path = request.url.path
    open_paths = ('/api/auth/status', '/api/auth/login')
    if (AUTH_TOKEN and request.method != 'OPTIONS' and path.startswith('/api')
            and path not in open_paths):
        if not _token_ok(_token_from_request(request)):
            return JSONResponse(
                status_code=401,
                content={'detail': '未授权：请先登录'},
                headers={
                    'WWW-Authenticate': 'Bearer',
                    'Access-Control-Allow-Origin': '*',
                    'Access-Control-Allow-Headers': '*',
                    'Access-Control-Allow-Methods': '*',
                })
    return await call_next(request)


# 兼容旧缓存页面：?api=http://IP:5500 的旧 JS 会直接请求 /health、/points 等，
# 这里把已知 API 段自动补上 /api 前缀。
_API_ALIAS_SEGMENTS = {
    'health', 'status', 'points', 'annotations', 'missions', 'mission',
    'map', 'maps', 'workspaces', 'workspace', 'plans', 'tools', 'auth',
}


@app.middleware('http')
async def _legacy_api_alias(request: Request, call_next):
    path = request.url.path
    if path and not path.startswith('/api'):
        first = path.lstrip('/').split('/')[0]
        if first in _API_ALIAS_SEGMENTS:
            rewritten = '/api' + path
            request.scope['path'] = rewritten
            if request.scope.get('raw_path') is not None:
                request.scope['raw_path'] = rewritten.encode()
    return await call_next(request)


@app.middleware('http')
async def _no_cache_static(request: Request, call_next):
    response = await call_next(request)
    if request.url.path.endswith(('.html', '.js', '.css')):
        response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    return response


class LoginRequest(BaseModel):
    token: str = Field(min_length=1, max_length=256)


@app.get('/api/auth/status')
def auth_status(request: Request) -> dict:
    return {
        'required': bool(AUTH_TOKEN),
        'authenticated': (not AUTH_TOKEN) or _token_ok(_token_from_request(request)),
    }


@app.post('/api/auth/login')
def auth_login(payload: LoginRequest) -> dict:
    if not AUTH_TOKEN:
        return {'ok': True, 'required': False}
    if not _token_ok(payload.token.strip()):
        raise HTTPException(status_code=401, detail='令牌不正确')
    return {'ok': True, 'required': True}


@app.on_event('startup')
def _on_startup() -> None:
    try:
        start_ros()
    except Exception as exc:
        global ROS_ERROR
        ROS_ERROR = f'start_ros failed: {type(exc).__name__}: {exc}'


@app.on_event('shutdown')
def _on_shutdown() -> None:
    stop_ros()


@app.get('/api/health')
def health() -> dict:
    status_age = None
    if ros_node is not None and ros_node.status_time is not None:
        status_age = round(time.time() - ros_node.status_time, 2)
    return {
        'ok': True,
        'ros_ready': ros_ready(),
        'ros_error': ROS_ERROR,
        'status_age': status_age,
        'mission': mission_executor.snapshot() if mission_executor else None,
        'time': _now(),
    }


@app.get('/api/status')
def robot_status() -> dict:
    if ros_node is None or ros_node.status is None:
        return {'available': False, 'status': None}
    return {
        'available': True,
        'age': round(time.time() - (ros_node.status_time or 0), 2),
        'status': ros_node.status,
    }


# ------------------------------------------------------------------ points
@app.get('/api/points')
def get_points() -> dict:
    return read_points()


@app.post('/api/points')
def add_point(point: PointModel) -> dict:
    data = read_points()
    raw = point.normalised()
    points = data['points']
    match = None
    if raw['id'] is not None:
        match = next((p for p in points if p['id'] == raw['id']), None)
    if match is None:
        match = next((p for p in points if p['name'] == raw['name']), None)
    if match is not None:
        match.update({'name': raw['name'], 'x': raw['x'], 'y': raw['y'], 'yaw': raw['yaw']})
    else:
        points.append(raw)
    return write_points(PointsPayload(frame=data['frame'], points=[PointModel(**p) for p in points]))


@app.delete('/api/points/{point_id}')
def delete_point(point_id: int) -> dict:
    data = read_points()
    remaining = [p for p in data['points'] if p['id'] != point_id]
    if len(remaining) == len(data['points']):
        raise HTTPException(status_code=404, detail='点位不存在')
    return write_points(PointsPayload(frame=data['frame'], points=[PointModel(**p) for p in remaining]))


@app.post('/api/points/save')
def save_points(payload: PointsPayload) -> dict:
    return write_points(payload)


# ------------------------------------------------------------- annotations
@app.get('/api/annotations')
def get_annotations(map_name: Optional[str] = None) -> dict:
    data = read_annotations()
    if map_name:
        data['annotations'] = [a for a in data['annotations'] if a.get('map') in ('', map_name)]
    return data


@app.post('/api/annotations')
def add_annotation(annotation: AnnotationModel) -> dict:
    data = read_annotations()
    raw = annotation.normalised()
    annotations = data['annotations']
    match = None
    if raw['id'] is not None:
        match = next((a for a in annotations if a['id'] == raw['id']), None)
    if match is not None:
        match.update(raw)
    else:
        annotations.append(raw)
    return write_annotations(AnnotationsPayload(
        frame=data['frame'], annotations=[AnnotationModel(**a) for a in annotations]))


@app.put('/api/annotations/{annotation_id}')
def update_annotation(annotation_id: int, annotation: AnnotationModel) -> dict:
    data = read_annotations()
    raw = annotation.normalised()
    raw['id'] = annotation_id
    found = False
    annotations = []
    for item in data['annotations']:
        if item['id'] == annotation_id:
            annotations.append(raw)
            found = True
        else:
            annotations.append(item)
    if not found:
        raise HTTPException(status_code=404, detail='标注不存在')
    return write_annotations(AnnotationsPayload(
        frame=data['frame'], annotations=[AnnotationModel(**a) for a in annotations]))


@app.delete('/api/annotations/{annotation_id}')
def delete_annotation(annotation_id: int) -> dict:
    data = read_annotations()
    remaining = [a for a in data['annotations'] if a['id'] != annotation_id]
    if len(remaining) == len(data['annotations']):
        raise HTTPException(status_code=404, detail='标注不存在')
    return write_annotations(AnnotationsPayload(
        frame=data['frame'], annotations=[AnnotationModel(**a) for a in remaining]))


@app.post('/api/annotations/save')
def save_annotations(payload: AnnotationsPayload) -> dict:
    return write_annotations(payload)


# ------------------------------------------------------------- workspaces
class WorkspaceCreate(BaseModel):
    name: str = Field(min_length=1, max_length=64)


def _ws_points_file(workspace: dict) -> Path:
    return Path(workspace['directory']) / 'points.yaml'


def _ws_annotations_file(workspace: dict) -> Path:
    return Path(workspace['directory']) / 'annotations.yaml'


def _yaml_count(path: Path, key: str) -> int:
    if not path.exists():
        return 0
    try:
        with path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        return len(data.get(key) or [])
    except (OSError, yaml.YAMLError, TypeError):
        return 0


def _workspace_public(workspace: dict, active_id: Optional[int]) -> dict:
    plans = workspace_store.plans(workspace['id']) if workspace_store else []
    active_plan = workspace_store.active_plan(workspace['id']) if workspace_store else None
    points_file = (Path(active_plan['points_file']) if active_plan
                   else _ws_points_file(workspace))
    annotations_file = _ws_annotations_file(workspace)
    return {
        'id': workspace['id'],
        'name': workspace['name'],
        'slug': workspace['slug'],
        'map_yaml': workspace.get('map_yaml'),
        'map_pgm': workspace.get('map_pgm'),
        'prior_pcd': workspace.get('prior_pcd'),
        'is_default': workspace.get('is_default', False),
        'active': workspace['id'] == active_id,
        'directory': _rel(Path(workspace['directory'])),
        'points_file': _rel(points_file),
        'annotations_file': _rel(annotations_file),
        'point_count': _yaml_count(points_file, 'points'),
        'annotation_count': _yaml_count(annotations_file, 'annotations'),
        'plan_count': len(plans),
        'active_plan_id': active_plan['id'] if active_plan else None,
        'active_plan_name': active_plan['name'] if active_plan else None,
        'created_at': workspace.get('created_at'),
        'updated_at': workspace.get('updated_at'),
    }


@app.get('/api/workspaces')
def list_workspaces() -> dict:
    active = workspace_store.active() if workspace_store else None
    active_id = active['id'] if active else None
    return {
        'active_id': active_id,
        'workspaces': [_workspace_public(ws, active_id) for ws in workspace_store.list()],
    }


@app.get('/api/workspace')
def get_active_workspace() -> dict:
    active = workspace_store.active() if workspace_store else None
    if active is None:
        raise HTTPException(status_code=503, detail='工作区尚未初始化')
    return _workspace_public(active, active['id'])


@app.post('/api/workspaces')
def create_workspace(payload: WorkspaceCreate) -> dict:
    workspace = workspace_store.create(payload.name)
    active = workspace_store.active()
    return _workspace_public(workspace, active['id'] if active else None)


@app.post('/api/workspaces/{workspace_id}/activate')
def activate_workspace(workspace_id: int) -> dict:
    workspace = workspace_store.set_active(workspace_id)
    if workspace is None:
        raise HTTPException(status_code=404, detail='工作区不存在')
    warnings = _apply_workspace_map(workspace)
    return {
        'ok': True,
        'workspace': _workspace_public(workspace, workspace_id),
        'warnings': warnings,
        'restart_required': True,
        'message': '已切换工作区；地图和先验点云写入 map_config.yaml，重启导航栈后生效',
    }


@app.delete('/api/workspaces/{workspace_id}')
def delete_workspace(workspace_id: int) -> dict:
    if not workspace_store.delete(workspace_id):
        raise HTTPException(status_code=404, detail='工作区不存在')
    return {'ok': True}


@app.post('/api/workspace/map')
def bind_workspace_map(payload: MapLoadRequest) -> dict:
    active = workspace_store.active() if workspace_store else None
    if active is None:
        raise HTTPException(status_code=503, detail='工作区尚未初始化')
    pgm_path, yaml_path, _ = _resolve_map_paths(payload.pgm, payload.yaml)
    workspace = workspace_store.update(
        active['id'], map_yaml=_rel(yaml_path), map_pgm=_rel(pgm_path))
    return {
        'ok': True,
        'workspace': _workspace_public(workspace or active, active['id']),
        'message': '已设为当前工作区地图；重启导航栈后生效',
    }


@app.post('/api/workspaces/{workspace_id}/pcd')
async def upload_workspace_pcd(
    workspace_id: int,
    pcd: UploadFile = File(...)) -> dict:
    workspace = workspace_store.get(workspace_id)
    if workspace is None:
        raise HTTPException(status_code=404, detail='工作区不存在')
    data = await pcd.read()
    if not data:
        raise HTTPException(status_code=422, detail='PCD 文件为空')
    if len(data) > 512 * 1024 * 1024:
        raise HTTPException(status_code=413, detail='PCD 超过 512MB')
    name = _safe_upload_name(pcd.filename, '.pcd')
    if not name.lower().endswith('.pcd'):
        raise HTTPException(status_code=422, detail='文件扩展名必须是 .pcd')
    target_dir = Path(workspace['directory']) / 'prior_pcd'
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / name
    try:
        target.write_bytes(data)
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f'保存 PCD 失败: {exc}')
    updated = workspace_store.update(workspace_id, prior_pcd=_rel(target))
    active = workspace_store.active()
    return {
        'ok': True,
        'workspace': _workspace_public(updated or workspace, active['id'] if active else None),
        'message': 'PCD 已保存到工作区；重启导航栈后生效',
    }


# ------------------------------------------------------- inspection plans
class PlanCreate(BaseModel):
    name: str = Field(min_length=1, max_length=64)


class PlanRename(BaseModel):
    name: str = Field(min_length=1, max_length=64)


def _plan_public(plan: dict, active_plan_id: Optional[int]) -> dict:
    return {
        'id': plan['id'],
        'workspace_id': plan['workspace_id'],
        'name': plan['name'],
        'slug': plan['slug'],
        'is_default': plan['is_default'],
        'active': plan['id'] == active_plan_id,
        'points_file': _rel(Path(plan['points_file'])),
        'point_count': _yaml_count(Path(plan['points_file']), 'points'),
        'created_at': plan['created_at'],
        'updated_at': plan['updated_at'],
    }


@app.get('/api/plans')
def list_plans() -> dict:
    workspace = workspace_store.active() if workspace_store else None
    if workspace is None:
        raise HTTPException(status_code=503, detail='工作区尚未初始化')
    active_plan = workspace_store.active_plan(workspace['id'])
    return {
        'workspace_id': workspace['id'],
        'workspace_name': workspace['name'],
        'active_id': active_plan['id'] if active_plan else None,
        'plans': [_plan_public(plan, active_plan['id'] if active_plan else None)
                  for plan in workspace_store.plans(workspace['id'])],
    }


@app.post('/api/plans')
def create_plan(payload: PlanCreate) -> dict:
    workspace = workspace_store.active() if workspace_store else None
    if workspace is None:
        raise HTTPException(status_code=503, detail='工作区尚未初始化')
    plan = workspace_store.create_plan(workspace['id'], payload.name)
    active_plan = workspace_store.active_plan(workspace['id'])
    return _plan_public(plan, active_plan['id'] if active_plan else None)


@app.put('/api/plans/{plan_id}')
def rename_plan(plan_id: int, payload: PlanRename) -> dict:
    plan = workspace_store.plan(plan_id)
    workspace = workspace_store.active() if workspace_store else None
    if plan is None or workspace is None or plan['workspace_id'] != workspace['id']:
        raise HTTPException(status_code=404, detail='方案不存在（或不属于当前工作区）')
    updated = workspace_store.rename_plan(plan_id, payload.name)
    active_plan = workspace_store.active_plan(workspace['id'])
    return _plan_public(updated or plan, active_plan['id'] if active_plan else None)


@app.post('/api/plans/{plan_id}/activate')
def activate_plan(plan_id: int) -> dict:
    workspace = workspace_store.active() if workspace_store else None
    if workspace is None:
        raise HTTPException(status_code=503, detail='工作区尚未初始化')
    plan = workspace_store.plan(plan_id)
    if plan is None or plan['workspace_id'] != workspace['id']:
        raise HTTPException(status_code=404, detail='方案不存在（或不属于当前工作区）')
    workspace_store.set_active_plan(workspace['id'], plan_id)
    return {
        'ok': True,
        'workspace_id': workspace['id'],
        'plan': _plan_public(plan, plan_id),
        'message': '已切换巡检方案',
    }


@app.delete('/api/plans/{plan_id}')
def delete_plan(plan_id: int) -> dict:
    plan = workspace_store.plan(plan_id)
    workspace = workspace_store.active() if workspace_store else None
    if plan is None or workspace is None or plan['workspace_id'] != workspace['id']:
        raise HTTPException(status_code=404, detail='方案不存在（或不属于当前工作区）')
    if not workspace_store.delete_plan(plan_id):
        raise HTTPException(status_code=404, detail='方案不存在')
    return {'ok': True}


# ------------------------------------------------------------------ missions
@app.get('/api/missions')
def list_missions() -> list[dict]:
    return store.list(_active_workspace_id())


@app.post('/api/missions')
def create_mission(payload: MissionModel) -> dict:
    points = _validate_points([p.model_dump() for p in payload.points])
    return store.create(_check_text(payload.name, '任务名'), points, payload.loop,
                        _active_workspace_id())


@app.put('/api/missions/{mission_id}')
def update_mission(mission_id: int, payload: MissionModel) -> dict:
    points = _validate_points([p.model_dump() for p in payload.points])
    mission = store.update(mission_id, _check_text(payload.name, '任务名'), points,
                           payload.loop, _active_workspace_id())
    if mission is None:
        raise HTTPException(status_code=404, detail='任务不存在（或不属于当前工作区）')
    return mission


@app.delete('/api/missions/{mission_id}')
def delete_mission(mission_id: int) -> dict:
    if not store.delete(mission_id, _active_workspace_id()):
        raise HTTPException(status_code=404, detail='任务不存在（或不属于当前工作区）')
    return {'ok': True}


@app.get('/api/mission/history')
def mission_history(limit: int = 50) -> list[dict]:
    limit = max(1, min(int(limit), 200))
    return store.list_runs(limit, _active_workspace_id())


@app.delete('/api/mission/history')
def clear_mission_history() -> dict:
    return {'ok': True, 'deleted': store.clear_runs(_active_workspace_id())}


@app.get('/api/mission/state')
def mission_state() -> dict:
    if mission_executor is None:
        return {
            'state': 'idle', 'mission_id': None, 'name': '', 'points': [],
            'index': 0, 'loop': False, 'distance_remaining': None,
            'message': 'ROS 未就绪', 'started_at': None, 'updated_at': _now(),
        }
    return mission_executor.snapshot()


@app.post('/api/mission/start')
def start_mission(payload: StartMissionRequest) -> dict:
    if mission_executor is None:
        raise HTTPException(status_code=503, detail='ROS 未就绪，无法启动任务')
    workspace_id = _active_workspace_id()
    if payload.mission_id is not None:
        mission = store.get(payload.mission_id, workspace_id)
        if mission is None:
            raise HTTPException(status_code=404, detail='任务不存在（或不属于当前工作区）')
        points = mission['points']
        name = mission['name']
        loop = mission['loop']
        mission_id = mission['id']
    else:
        points = _validate_points([p.model_dump() for p in (payload.points or [])])
        name = _check_text(payload.name, '任务名')
        loop = payload.loop
        mission_id = None
    return mission_executor.start(name, points, loop, mission_id, workspace_id)


@app.post('/api/mission/pause')
def pause_mission() -> dict:
    if mission_executor is None:
        raise HTTPException(status_code=503, detail='ROS 未就绪')
    return mission_executor.pause()


@app.post('/api/mission/resume')
def resume_mission() -> dict:
    if mission_executor is None:
        raise HTTPException(status_code=503, detail='ROS 未就绪')
    return mission_executor.resume()


@app.post('/api/mission/cancel')
def cancel_mission() -> dict:
    if mission_executor is None:
        raise HTTPException(status_code=503, detail='ROS 未就绪')
    return mission_executor.cancel()


@app.post('/api/mission/reset')
def reset_mission() -> dict:
    if mission_executor is None:
        raise HTTPException(status_code=503, detail='ROS 未就绪')
    return mission_executor.reset()


# ------------------------------------------------------------------ maps
class MapLoadRequest(BaseModel):
    pgm: Optional[str] = None
    yaml: Optional[str] = None
    name: Optional[str] = None


class SaveMapRequest(BaseModel):
    name: str
    directory: Optional[str] = None


class Pcd2GridRequest(BaseModel):
    pcd: str
    output: Optional[str] = None
    resolution: float = 0.05
    # 默认值适配当前 0.15 m 降采样的 PCD；如果 PCD 更密，可以调高命中阈值。
    ground_min: float = -0.35
    ground_max: float = 0.15
    obstacle_min: float = 0.20
    obstacle_max: float = 1.50
    min_ground_hits: int = 1
    min_obstacle_hits: int = 2


def _read_map_yaml(yaml_path: Path) -> dict:
    try:
        with yaml_path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
    except OSError as exc:
        raise HTTPException(status_code=404, detail=f'读取 YAML 失败: {exc}')
    except yaml.YAMLError as exc:
        raise HTTPException(status_code=422, detail=f'YAML 解析失败: {exc}')
    if not isinstance(data, dict) or 'resolution' not in data or 'image' not in data:
        raise HTTPException(status_code=422, detail='这个 YAML 不是地图配置（缺少 image/resolution）')
    return data


def _resolve_map_paths(pgm: Optional[str], yaml_file: Optional[str]) -> tuple[Path, Optional[Path], dict]:
    """返回 (pgm 路径, yaml 路径, yaml 元数据)。至少提供其中一个。"""
    if not pgm and not yaml_file:
        raise HTTPException(status_code=422, detail='请提供 .pgm 或 .yaml 文件')
    yaml_path: Optional[Path] = None
    meta: dict = {}
    if yaml_file:
        yaml_path = _safe_path(yaml_file, WORKSPACE)
        if not yaml_path.exists() or yaml_path.suffix.lower() not in ('.yaml', '.yml'):
            raise HTTPException(status_code=404, detail=f'YAML 不存在或不是 .yaml: {yaml_path}')
        meta = _read_map_yaml(yaml_path)
    if pgm:
        pgm_path = _safe_path(pgm, WORKSPACE)
    else:
        image_ref = str(meta.get('image') or '')
        if not image_ref:
            raise HTTPException(status_code=422, detail='YAML 缺少 image 字段')
        pgm_path = _safe_path((yaml_path.parent / image_ref), WORKSPACE)
    if not pgm_path.exists() or pgm_path.suffix.lower() != '.pgm':
        raise HTTPException(status_code=404, detail=f'PGM 不存在或不是 .pgm: {pgm_path}')
    if yaml_path is None:
        candidate = pgm_path.with_suffix('.yaml')
        if not candidate.exists():
            candidate = pgm_path.with_suffix('.yml')
        if candidate.exists():
            yaml_path = candidate
            meta = _read_map_yaml(yaml_path)
    return pgm_path, yaml_path, meta


def _map_render_params(meta: dict) -> tuple[int, float, float]:
    negate = int(meta.get('negate', 0) or 0)
    occupied = float(meta.get('occupied_thresh', 0.65))
    free = float(meta.get('free_thresh', 0.196))
    return negate, occupied, free


MAP_IMAGE_CACHE: dict[tuple, bytes] = {}
MAP_CACHE_LOCK = threading.Lock()


def _render_map_png(pgm_path: Path, negate: int, occupied_thresh: float,
                    free_thresh: float) -> bytes:
    """把 PGM 按 ROS map_server 语义渲染成 PNG（自由=浅灰、占据=深灰、未知=中灰）。"""
    try:
        from PIL import Image
        import numpy as np
    except Exception as exc:
        raise HTTPException(status_code=503, detail=f'缺少 Pillow/numpy: {exc}')
    stat = pgm_path.stat()
    key = (str(pgm_path.resolve()), stat.st_mtime_ns, stat.st_size,
           negate, round(occupied_thresh, 6), round(free_thresh, 6))
    with MAP_CACHE_LOCK:
        cached = MAP_IMAGE_CACHE.get(key)
    if cached:
        return cached
    try:
        with Image.open(pgm_path) as image:
            gray = image.convert('L')
            arr = np.asarray(gray, dtype=np.float32)
    except Exception as exc:
        raise HTTPException(status_code=422, detail=f'PGM 读取失败: {exc}')
    # map_server: occ = (255 - color) / 255；negate=1 时取反
    occ = (255.0 - arr) / 255.0
    if negate:
        occ = 1.0 - occ
    rgb = np.empty(arr.shape + (3,), dtype=np.uint8)
    rgb[:, :] = (200, 200, 200)
    free_mask = occ < free_thresh
    occupied_mask = occ > occupied_thresh
    rgb[free_mask] = (242, 242, 242)
    rgb[occupied_mask] = (43, 43, 43)
    buffer = io.BytesIO()
    Image.fromarray(rgb, 'RGB').save(buffer, format='PNG', optimize=False)
    data = buffer.getvalue()
    with MAP_CACHE_LOCK:
        if len(MAP_IMAGE_CACHE) > 8:
            MAP_IMAGE_CACHE.clear()
        MAP_IMAGE_CACHE[key] = data
    return data


def _scan_map_sources(limit: int = 200) -> list[dict]:
    roots = [MAP_DIR, WORKSPACE / 'runs', UPLOAD_MAP_DIR, WORKSPACES_DIR]
    seen = set()
    sources = []
    for root in roots:
        if not root.exists():
            continue
        for path in list(root.rglob('*.yaml')) + list(root.rglob('*.yml')):
            resolved = path.resolve()
            if resolved in seen:
                continue
            seen.add(resolved)
            try:
                with path.open(encoding='utf-8') as stream:
                    data = yaml.safe_load(stream) or {}
            except (OSError, yaml.YAMLError):
                continue
            if not isinstance(data, dict) or 'image' not in data or 'resolution' not in data:
                continue
            try:
                image_path = (path.parent / str(data['image'])).resolve()
            except (OSError, ValueError):
                continue
            sources.append({
                'yaml': _rel(path),
                'pgm': _rel(image_path),
                'name': path.stem,
                'resolution': float(data.get('resolution', 0.0) or 0.0),
                'mtime': datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec='seconds'),
            })
    sources.sort(key=lambda item: item['mtime'], reverse=True)
    return sources[:limit]


@app.get('/api/map/sources')
def map_sources() -> list[dict]:
    return _scan_map_sources()


@app.post('/api/map/load')
def load_map(payload: MapLoadRequest) -> dict:
    pgm_path, yaml_path, meta = _resolve_map_paths(payload.pgm, payload.yaml)
    try:
        from PIL import Image
        with Image.open(pgm_path) as image:
            width, height = image.size
    except Exception as exc:
        raise HTTPException(status_code=422, detail=f'PGM 读取失败: {exc}')
    resolution = float(meta.get('resolution', 0.0) or 0.0)
    if resolution <= 0:
        raise HTTPException(status_code=422, detail='YAML resolution 必须大于 0')
    origin = meta.get('origin') or [0.0, 0.0, 0.0]
    if not isinstance(origin, list) or len(origin) < 3:
        raise HTTPException(status_code=422, detail='YAML origin 必须是 [x, y, yaw]')
    negate, occupied, free = _map_render_params(meta)
    query = 'yaml=' + str(yaml_path.resolve().relative_to(WORKSPACE)) if yaml_path else 'pgm=' + _rel(pgm_path)
    from urllib.parse import quote

    warnings = []
    image_ref = str(meta.get('image') or '')
    if image_ref and yaml_path is not None:
        declared = (yaml_path.parent / image_ref).resolve()
        if declared != pgm_path.resolve():
            warnings.append(f'YAML 指向的 image 是 {_rel(declared)}，当前使用你填写的 {_rel(pgm_path)}')
    return {
        'ok': True,
        'yaml': _rel(yaml_path) if yaml_path else None,
        'pgm': _rel(pgm_path),
        'name': payload.name or (yaml_path.stem if yaml_path else pgm_path.stem),
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin': [float(origin[0]), float(origin[1]), float(origin[2])],
        'negate': negate,
        'occupied_thresh': occupied,
        'free_thresh': free,
        'image_url': '/api/map/image?' + quote(query, safe='=/'),
        'warnings': warnings,
    }


def _safe_upload_name(name: Optional[str], suffix: str) -> str:
    base = Path(name or '').name
    base = re.sub(r'[^A-Za-z0-9_.-]', '_', base)
    if not base:
        base = 'map'
    if not base.lower().endswith(suffix):
        base += suffix
    return base[:96]


def _prune_upload_dirs(directory: Path, keep: int = 20) -> None:
    if not directory.exists():
        return
    try:
        dirs = sorted(
            [item for item in directory.iterdir() if item.is_dir()],
            key=lambda item: item.stat().st_mtime, reverse=True)
        for old in dirs[keep:]:
            shutil.rmtree(old, ignore_errors=True)
    except OSError:
        pass


@app.post('/api/map/upload')
async def upload_map(
    pgm: UploadFile = File(...),
    yaml_file: UploadFile = File(...)) -> dict:
    """接收浏览器选中的 PGM+YAML，保存到当前工作区并绑定为工作区地图。"""
    pgm_bytes = await pgm.read()
    yaml_bytes = await yaml_file.read()
    if not pgm_bytes or not yaml_bytes:
        raise HTTPException(status_code=422, detail='PGM 或 YAML 文件为空')
    if len(pgm_bytes) > 64 * 1024 * 1024:
        raise HTTPException(status_code=413, detail='PGM 文件超过 64MB')
    pgm_name = _safe_upload_name(pgm.filename, '.pgm')
    if not pgm_name.lower().endswith('.pgm'):
        raise HTTPException(status_code=422, detail='PGM 文件扩展名必须是 .pgm')
    try:
        text = yaml_bytes.decode('utf-8-sig')
    except UnicodeDecodeError:
        text = yaml_bytes.decode('latin-1', errors='replace')
    try:
        meta = yaml.safe_load(text) or {}
    except yaml.YAMLError as exc:
        raise HTTPException(status_code=422, detail=f'YAML 解析失败: {exc}')
    if not isinstance(meta, dict) or 'resolution' not in meta:
        raise HTTPException(status_code=422, detail='YAML 不是地图配置（缺少 resolution）')
    # 上传目录里的文件名可能与 YAML 原 image 不一致，统一改写成实际保存的 PGM 名
    meta['image'] = pgm_name
    yaml_name = _safe_upload_name(yaml_file.filename, '.yaml')
    workspace = workspace_store.active() if workspace_store else None
    root = (Path(workspace['directory']) / 'maps') if workspace else UPLOAD_MAP_DIR
    target = root / f'{int(time.time() * 1000)}_{secrets.token_hex(4)}'
    try:
        target.mkdir(parents=True, exist_ok=False)
        (target / pgm_name).write_bytes(pgm_bytes)
        with (target / yaml_name).open('w', encoding='utf-8') as stream:
            yaml.safe_dump(meta, stream, allow_unicode=True, sort_keys=False)
    except OSError as exc:
        shutil.rmtree(target, ignore_errors=True)
        raise HTTPException(status_code=500, detail=f'保存上传文件失败: {exc}')
    _prune_upload_dirs(root)
    if workspace and workspace_store is not None:
        workspace_store.update(
            workspace['id'],
            map_yaml=_rel(target / yaml_name),
            map_pgm=_rel(target / pgm_name))
    return {
        'ok': True,
        'workspace_id': workspace['id'] if workspace else None,
        'name': Path(yaml_name).stem,
        'yaml': _rel(target / yaml_name),
        'pgm': _rel(target / pgm_name),
    }


@app.get('/api/map/image')
def map_image(yaml: Optional[str] = None, pgm: Optional[str] = None) -> Response:
    pgm_path, _, meta = _resolve_map_paths(pgm, yaml)
    negate, occupied, free = _map_render_params(meta)
    data = _render_map_png(pgm_path, negate, occupied, free)
    return Response(content=data, media_type='image/png',
                    headers={'Cache-Control': 'no-cache'})


@app.get('/api/map/files')
def map_files() -> list[dict]:
    if not MAP_DIR.exists():
        return []
    files = []
    for path in sorted(MAP_DIR.glob('*')):
        if path.suffix.lower() not in ('.yaml', '.pgm', '.pcd', '.png'):
            continue
        stat = path.stat()
        files.append({
            'name': path.name,
            'size': stat.st_size,
            'mtime': datetime.fromtimestamp(stat.st_mtime).isoformat(timespec='seconds'),
        })
    return files


@app.post('/api/map/save2d')
def save_map2d(payload: SaveMapRequest) -> dict:
    name = _check_name(payload.name)
    target_dir = MAP_DIR if payload.directory is None else _safe_path(payload.directory, WORKSPACE)
    target_dir.mkdir(parents=True, exist_ok=True)
    output = target_dir / name
    ros2 = shutil.which('ros2')
    if ros2 is None:
        return {'ok': False, 'message': '找不到 ros2 命令，请先 source ROS 环境',
                'stdout': '', 'stderr': '', 'path': str(output)}
    command = [ros2, 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(output)]
    try:
        proc = subprocess.run(
            command, cwd=str(WORKSPACE), capture_output=True, text=True,
            timeout=45, env=os.environ.copy())
    except subprocess.TimeoutExpired:
        return {'ok': False, 'message': 'map_saver_cli 超时（45s）',
                'stdout': '', 'stderr': '', 'path': str(output)}
    ok = proc.returncode == 0 and output.with_suffix('.yaml').exists()
    return {
        'ok': ok,
        'message': '二维地图已保存' if ok else '保存失败，检查 /map 是否在发布',
        'stdout': proc.stdout[-4000:],
        'stderr': proc.stderr[-4000:],
        'path': str(output.with_suffix('.yaml')),
    }


@app.post('/api/map/save3d')
def save_map3d() -> dict:
    return {
        'ok': False,
        'message': '三维 PCD 由 Super-LIO 在退出建图时保存；后端不会强制停止建图进程。',
        'path': '',
    }


@app.post('/api/tools/pcd2grid')
def pcd_to_grid(payload: Pcd2GridRequest) -> dict:
    pcd = _safe_path(payload.pcd, WORKSPACE)
    if not pcd.exists() or pcd.suffix.lower() != '.pcd':
        raise HTTPException(status_code=404, detail=f'PCD 文件不存在或不是 .pcd: {pcd}')
    if payload.output:
        output = _safe_path(payload.output, WORKSPACE)
    else:
        output = MAP_DIR / f'{pcd.stem}_grid'
    output.parent.mkdir(parents=True, exist_ok=True)
    script = WORKSPACE / 'scripts' / 'pcd_to_occupancy_map.py'
    command = [
        sys.executable, str(script), str(pcd), str(output),
        '--resolution', str(float(payload.resolution)),
        '--ground-min', str(float(payload.ground_min)),
        '--ground-max', str(float(payload.ground_max)),
        '--obstacle-min', str(float(payload.obstacle_min)),
        '--obstacle-max', str(float(payload.obstacle_max)),
        '--min-ground-hits', str(int(payload.min_ground_hits)),
        '--min-obstacle-hits', str(int(payload.min_obstacle_hits)),
    ]
    try:
        proc = subprocess.run(
            command, cwd=str(WORKSPACE), capture_output=True, text=True,
            timeout=180, env=os.environ.copy())
    except subprocess.TimeoutExpired:
        return {'ok': False, 'message': 'PCD 转栅格超时（180s）', 'stdout': '', 'stderr': ''}
    ok = proc.returncode == 0
    return {
        'ok': ok,
        'message': '栅格地图生成完成' if ok else '生成失败',
        'stdout': proc.stdout[-4000:],
        'stderr': proc.stderr[-4000:],
        'path': str(output.with_suffix('.yaml')),
    }


# ------------------------------------------------------------------ static
if WEB_ROOT.is_dir():
    app.mount('/', StaticFiles(directory=str(WEB_ROOT), html=True), name='web')


def main() -> None:
    global AUTH_TOKEN
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5500)
    parser.add_argument('--web-root', default=str(WEB_ROOT))
    parser.add_argument(
        '--token', default=os.environ.get('BUNKER_WEB_TOKEN', ''),
        help='API 访问令牌；留空则不启用鉴权，也可用 BUNKER_WEB_TOKEN 环境变量')
    args = parser.parse_args()
    AUTH_TOKEN = (args.token or '').strip()
    if AUTH_TOKEN:
        print('API 鉴权已启用（Bearer token）', flush=True)
    import uvicorn
    uvicorn.run(app, host=args.host, port=args.port, log_level='info')


if __name__ == '__main__':
    main()
