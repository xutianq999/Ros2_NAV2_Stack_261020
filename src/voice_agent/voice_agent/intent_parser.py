"""小程序语音指令的确定性解析器。

设计原则：
- 语音链路不能把“停止/急停”完全交给大模型判断，因此运动、导航、停止类
  指令先用规则解析，命中后直接执行；
- 规则只处理明确的口令。带疑问词、同时出现多个方向、或语义含糊的句子
  返回 None，交给大模型（或前端提示）处理，避免误动作。
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ParsedCommand:
    """一条已识别的机器人指令。"""

    action: str
    direction: Optional[str] = None
    distance_m: Optional[float] = None
    angle_deg: Optional[float] = None
    speed_scale: float = 1.0
    point: Optional[str] = None
    raw: str = ""


_CN_DIGITS = {
    "零": 0, "〇": 0, "一": 1, "壹": 1, "二": 2, "贰": 2, "两": 2,
    "三": 3, "叁": 3, "四": 4, "肆": 4, "五": 5, "伍": 5, "六": 6,
    "陆": 6, "七": 7, "柒": 7, "八": 8, "捌": 8, "九": 9, "玖": 9,
}
_CN_NUMERAL_RE = re.compile(r"[零〇一壹二贰两三四五六七八九十百]+")
_FULLWIDTH = str.maketrans(
    "０１２３４５６７８９．，：；！？", "0123456789.,:;!?"
)


def _cn_to_int(text: str) -> Optional[int]:
    """把中文数字（十以内、以及几十/几百）转换成整数。"""
    if not text:
        return None
    if text.isdigit():
        return int(text)
    if text == "十":
        return 10
    if "百" in text:
        left, _, right = text.partition("百")
        hundreds = _CN_DIGITS.get(left, 1) if left else 1
        rest = _cn_to_int(right) if right else 0
        return hundreds * 100 + (rest or 0)
    if "十" in text:
        left, _, right = text.partition("十")
        tens = _CN_DIGITS.get(left, 1) if left else 1
        ones = _CN_DIGITS.get(right, 0) if right else 0
        return tens * 10 + ones
    if len(text) == 1:
        return _CN_DIGITS.get(text)
    # “一二”这类逐位数字（如年份）不在口令里使用，直接拼接
    if all(ch in _CN_DIGITS for ch in text):
        return int("".join(str(_CN_DIGITS[ch]) for ch in text))
    return None


def normalize(text: str) -> str:
    """统一全角/半角、去空格和标点，并把中文数字换成阿拉伯数字。"""
    text = text.strip().translate(_FULLWIDTH).lower()
    text = re.sub(r"[\s,，。！？!?;；:：、\"'“”‘’]+", "", text)

    def _replace(match: re.Match[str]) -> str:
        value = _cn_to_int(match.group(0))
        return str(value) if value is not None else match.group(0)

    return _CN_NUMERAL_RE.sub(_replace, text)


def _has_question_tone(raw: str) -> bool:
    return bool(re.search(r"(怎么|如何|为什么|是什么|哪个|哪些|能不能|可不可以|吗|呢)", raw))


_STOP_RE = re.compile(
    r"^(?:先|请|麻烦|帮我|快|马上|立即|赶紧|给我|把车|让车)*"
    r"(?:停止|停下|停下来|停车|停|刹车|站住|别动|别走|不要动|不要走|stop|halt|freeze)"
    r"(?:了|吧|呀|啊|一下|1下)?$"
)
# “别前进”“停止右转”这类否定式也按停止处理，避免被方向词误判成运动指令
_STOP_WITH_DIRECTION_RE = re.compile(
    r"^(?:先|请|麻烦|帮我|快|马上|立即|赶紧|给我|把车|让车)*"
    r"(?:停止|停下|别|不要|不用|别再)"
    r"(?:继续)?(?:前进|向前|往前|后退|向后|左转|右转|移动|走|动)"
    r"(?:了|吧|呀|啊|一下|1下)?$"
)
_ESTOP_RE = re.compile(
    r"^(?:请|麻烦|帮我|快|马上|立即|赶紧|给我|先|把车|让车)*"
    r"(?:急停|紧急停止|紧急停车|emergencystop|estop)"
    r"(?:吧|呀|啊|一下|1下)?$"
)
_RELEASE_ESTOP_RE = re.compile(
    r"^(?:请|麻烦|帮我|快|马上|立即|赶紧|给我|先)*"
    r"(?:解除急停|取消急停|恢复行驶|恢复|releaseestop|resetestop)"
    r"(?:吧|呀|啊|一下|1下)?$"
)
_CANCEL_NAV_RE = re.compile(
    r"^(?:请|麻烦|帮我|快|马上|立即|赶紧|给我|先)*"
    r"(?:取消导航|停止导航|终止导航|取消目标|停止前往|cancelnavigation|stopnavigation)"
    r"(?:吧|呀|啊|一下|1下)?$"
)

_NAV_RE = re.compile(
    r"^(?:请|麻烦|帮我|带我|我想|我要|能不能|可以)?"
    r"(?:导航到|导航去|导航|去|到|前往|开到|走到|返回|回到|goto|navigateto|navigate|takemeto)"
    r"(.+)$"
)

_FORWARD_RE = re.compile(r"(前进|向前|往前|直行|往前走|向前走|forward|goforward|moveforward|ahead)")
_BACKWARD_RE = re.compile(r"(后退|向后|往后|倒车|倒退|back|backward|reverse|goback)")
_LEFT_RE = re.compile(r"(左转|向左转|往左转|向左|往左|左拐|turnleft|left)")
_RIGHT_RE = re.compile(r"(右转|向右转|往右转|向右|往右|右拐|turnright|right)")
_STRAFE_RE = re.compile(r"(左移|右移|向左平移|向右平移|平移|横移|strafe|sidestep)")

_LIST_RE = re.compile(
    r"^(?:请|麻烦|帮我|给我|先)?(?:有哪些点|有哪些点位|点位列表|航点列表|有哪些航点|能去哪里|"
    r"listpoints|listwaypoints|waypoints)(?:吧|呀|啊|一下|1下)?$"
)
_STATUS_RE = re.compile(
    r"^(?:请|麻烦|帮我|给我|先)?(?:状态|机器人状态|小车状态|电量|电池|位置|你在哪|你在哪里|"
    r"现在怎么样|robotstatus|status|whereareyou|battery)(?:吧|呀|啊|怎么样)?$"
)
_HELP_RE = re.compile(
    r"^(?:请|麻烦|帮我|给我|先)?(?:帮助|能做什么|会做什么|怎么用|使用说明|help|commands)"
    r"(?:吧|呀|啊|一下|1下)?$"
)

# 距离/角度：先处理“一米五”=1.5 米这种口语表达
_SPOKEN_DECIMAL_RE = re.compile(r"(\d+)米([0-9])(?!\d)")
_DISTANCE_RE = re.compile(r"(\d+(?:\.\d+)?|半)(毫米|mm|厘米|cm|米|m)")
_ANGLE_RE = re.compile(r"(\d+(?:\.\d+)?|半)(度|°|deg)")
_SPEED_RE = re.compile(r"(?:速度|以)?(\d+(?:\.\d+)?)(?:米每秒|米/秒|m/s)")


def _extract_distance(text: str) -> Optional[float]:
    text = _SPOKEN_DECIMAL_RE.sub(r"\1.\2米", text)
    match = _DISTANCE_RE.search(text)
    if not match:
        return None
    value_text, unit = match.group(1), match.group(2)
    value = 0.5 if value_text == "半" else float(value_text)
    if unit in ("毫米", "mm"):
        value /= 1000.0
    elif unit in ("厘米", "cm"):
        value /= 100.0
    return value


def _extract_angle(text: str) -> Optional[float]:
    match = _ANGLE_RE.search(text)
    if not match:
        return None
    value_text = match.group(1)
    return 0.5 * 360.0 if value_text == "半" else float(value_text)


def _extract_speed_scale(text: str) -> float:
    scale = 1.0
    if re.search(r"(慢一点|慢点|慢慢|放慢|slow)", text):
        scale = 0.6
    if re.search(r"(快一点|快点|加快|快速|fast|quick)", text):
        scale = 1.3
    return scale


def _count_direction_groups(text: str) -> int:
    return sum(
        bool(regex.search(text))
        for regex in (_FORWARD_RE, _BACKWARD_RE, _LEFT_RE, _RIGHT_RE)
    )


def _clean_point_query(query: str) -> str:
    query = re.sub(r"(吧|呀|啊|谢谢|请|一下)$", "", query)
    query = re.sub(r"^(?:到|去)", "", query)
    return query.strip()


def parse_command(text: str) -> Optional[ParsedCommand]:
    """解析一条文本口令；不是明确机器人指令时返回 None。"""
    if not text or not text.strip():
        return None

    raw = text.strip()
    norm = normalize(raw)

    # 1. 最高优先级：急停 / 解除急停 / 取消导航
    if _ESTOP_RE.match(norm):
        return ParsedCommand(action="estop", raw=raw)
    if _RELEASE_ESTOP_RE.match(norm):
        return ParsedCommand(action="release_estop", raw=raw)
    if _CANCEL_NAV_RE.match(norm):
        return ParsedCommand(action="cancel_nav", raw=raw)
    if _STOP_RE.match(norm) or _STOP_WITH_DIRECTION_RE.match(norm):
        return ParsedCommand(action="stop", raw=raw)

    # 2. 查询类
    if _LIST_RE.match(norm):
        return ParsedCommand(action="list_waypoints", raw=raw)
    if _STATUS_RE.match(norm):
        return ParsedCommand(action="status", raw=raw)
    if _HELP_RE.match(norm):
        return ParsedCommand(action="help", raw=raw)

    # 3. 导航
    nav_match = _NAV_RE.match(norm)
    if nav_match:
        query = _clean_point_query(nav_match.group(1))
        # “到底怎么回事”“到哪里”这类问句不是导航指令
        if query and not re.match(
            r"^(底|哪|什么|怎么|为什么|谁|干嘛|干什么|哪里)", query
        ):
            return ParsedCommand(action="navigate", point=query, raw=raw)

    # 4. 运动。带疑问语气的句子交给大模型，避免“左转还是右转”被执行。
    if _has_question_tone(raw):
        return None
    if _STRAFE_RE.search(norm):
        return ParsedCommand(action="unsupported", raw=raw)

    direction_groups = _count_direction_groups(norm)
    if direction_groups != 1:
        return None

    speed_scale = _extract_speed_scale(raw)
    if _FORWARD_RE.search(norm):
        return ParsedCommand(
            action="move", direction="forward",
            distance_m=_extract_distance(norm), speed_scale=speed_scale, raw=raw,
        )
    if _BACKWARD_RE.search(norm):
        return ParsedCommand(
            action="move", direction="backward",
            distance_m=_extract_distance(norm), speed_scale=speed_scale, raw=raw,
        )
    if _LEFT_RE.search(norm):
        return ParsedCommand(
            action="move", direction="left",
            angle_deg=_extract_angle(norm), speed_scale=speed_scale, raw=raw,
        )
    if _RIGHT_RE.search(norm):
        return ParsedCommand(
            action="move", direction="right",
            angle_deg=_extract_angle(norm), speed_scale=speed_scale, raw=raw,
        )

    return None
