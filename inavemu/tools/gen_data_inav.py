#!/usr/bin/env python3
"""
Генератор данных для виртуального полётного контроллера INAV (inavemu).

Разбирает исходники INAV и вытаскивает:
  * таблицу настроек (settings.yaml: имя, тип, диапазон / lookup-таблица, секция, PGN);
  * lookup-таблицы значений;
  * список фич (из конфигуратора: js/fc.js — смысл битов mask `MSP_FEATURE`);
  * список боксов (режимов) из fc/fc_msp_box.c (permanentId для `MSP_BOXIDS`);
  * функции серийных портов (из конфигуратора: js/serialPortHelper.js).

Типы полей, не указанные в settings.yaml напрямую, определяются компилятором gcc
так же, как это делает src/utils/settings.rb в исходниках INAV: поле «тыкается»
через `.__type_detect_`, и gcc в сообщении об ошибке называет его тип.

Использование:
    python3 tools/gen_data_inav.py /path/to/inav [-o inavemu/data/inav_data.json]
    python3 tools/gen_data_inav.py /path/to/inav --configurator /path/to/inav-configurator

Ничего, кроме stdlib и gcc, не требуется.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

# --------------------------------------------------------------------------------------
# мини-парсер YAML (подмножество settings.yaml: вложенные словари/списки, скаляры,
# inline-списки ["A", "B"]; булевы значения по YAML 1.1: OFF/ON/yes/no/true/false)
# --------------------------------------------------------------------------------------

_YAML_BOOLS = {
    "y": True, "yes": True, "true": True, "on": True,
    "n": False, "no": False, "false": False, "off": False,
}

_INT_RE = re.compile(r"^[+-]?\d+$")
_HEX_RE = re.compile(r"^0[xX][0-9a-fA-F]+$")
_FLOAT_RE = re.compile(r"^[+-]?\d+\.\d+([eE][+-]?\d+)?$")


def _scalar(text: str):
    s = text.strip()
    if s == "" or s == "~" or s.lower() == "null":
        return None
    if (s.startswith('"') and s.endswith('"') and len(s) >= 2) or (
        s.startswith("'") and s.endswith("'") and len(s) >= 2
    ):
        body = s[1:-1]
        if s[0] == '"':
            body = body.replace('\\"', '"').replace("\\n", "\n").replace("\\\\", "\\")
        else:
            body = body.replace("''", "'")
        return body
    low = s.lower()
    if low in _YAML_BOOLS:
        return _YAML_BOOLS[low]
    if _INT_RE.match(s):
        return int(s)
    if _HEX_RE.match(s):
        return int(s, 16)
    if _FLOAT_RE.match(s):
        return float(s)
    return s


def _inline_list(text: str):
    body = text.strip()[1:-1].strip()
    if not body:
        return []
    items, cur, quote = [], "", None
    for ch in body:
        if quote:
            cur += ch
            if ch == quote:
                quote = None
            continue
        if ch in "\"'":
            quote = ch
            cur += ch
        elif ch == ",":
            items.append(cur)
            cur = ""
        else:
            cur += ch
    if cur.strip():
        items.append(cur)
    return [_scalar(i) for i in items]


def parse_yaml(text: str):
    """Разбирает подмножество YAML, используемое в settings.yaml."""
    lines = []
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        lines.append((indent, raw.strip()))
    pos = [0]

    def peek():
        return lines[pos[0]] if pos[0] < len(lines) else None

    def parse_value(indent):
        # значение на той же строке? (вызывается, когда после "key:" ничего нет)
        item = peek()
        if item is None or item[0] < indent:
            return None
        if item[0] > indent:
            return parse_block(item[0])
        # скаляр остался на этой строке — обрабатывается в parse_block
        return parse_block(item[0])

    def parse_block(indent):
        container = None
        while True:
            item = peek()
            if item is None or item[0] < indent:
                break
            ind, content = item
            if ind > indent:
                # глубже — дочерний блок для последнего ключа
                child = parse_block(ind)
                if isinstance(container, list):
                    container.append(child)
                elif isinstance(container, dict) and container:
                    last = list(container)[-1]
                    if container[last] is None:
                        container[last] = child
                continue
            if content.startswith("- ") or content == "-":
                if container is None:
                    container = []
                elif not isinstance(container, list):
                    break
                pos[0] += 1
                rest = content[2:].strip() if content != "-" else ""
                if rest == "":
                    container.append(parse_value(ind + 2))
                elif ":" in rest and not rest.startswith(('"', "'")):
                    # элемент-словарь: "- key: value" (+ возможные дочерние строки)
                    key, _, val = rest.partition(":")
                    d = {}
                    v = val.strip()
                    if v == "":
                        sub = peek()
                        if sub is not None and (
                            sub[0] > ind or (sub[0] == ind and sub[1].startswith("- "))
                        ):
                            d[key.strip()] = parse_block(sub[0] if sub[0] > ind else ind)
                        else:
                            d[key.strip()] = None
                    elif v.startswith("[") and v.endswith("]"):
                        d[key.strip()] = _inline_list(v)
                    else:
                        d[key.strip()] = _scalar(v)
                    # продолжение словаря на больших отступах
                    sub = peek()
                    if sub is not None and sub[0] > ind and not sub[1].startswith("- "):
                        more = parse_block(sub[0])
                        if isinstance(more, dict):
                            d.update(more)
                    container.append(d)
                elif ":" in rest:
                    container.append(_scalar(rest))
                else:
                    container.append(_scalar(rest))
            else:
                if container is None:
                    container = {}
                elif isinstance(container, list):
                    break
                key, _, val = content.partition(":")
                key = key.strip()
                v = val.strip()
                pos[0] += 1
                if v == "":
                    sub = peek()
                    if sub is not None and (
                        sub[0] > ind or (sub[0] == ind and sub[1].startswith("- "))
                    ):
                        container[key] = parse_block(sub[0] if sub[0] > ind else ind)
                    else:
                        container[key] = None
                elif v.startswith("[") and v.endswith("]"):
                    container[key] = _inline_list(v)
                else:
                    container[key] = _scalar(v)
        return container if container is not None else {}

    result = parse_block(lines[0][0] if lines else 0)
    return result


# --------------------------------------------------------------------------------------
# константы из заголовков (#define, enum-элементы) и вычисление выражений
# --------------------------------------------------------------------------------------

DEFINE_RE = re.compile(r"^\s*#\s*define\s+(\w+)\s+(.+?)\s*(?://.*)?$")
ENUM_ENTRY_RE = re.compile(r"^\s*(\w+)\s*(?:=\s*(.+?))?\s*,?\s*(?://.*)?$")


def _strip_c_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//.*?$", "", text, flags=re.M)
    return text


def collect_constants(repo: Path) -> dict:
    consts = {
        "INT8_MIN": -128, "INT8_MAX": 127, "UINT8_MAX": 255,
        "INT16_MIN": -32768, "INT16_MAX": 32767, "UINT16_MAX": 65535,
        "INT32_MIN": -2147483648, "INT32_MAX": 2147483647, "UINT32_MAX": 4294967295,
        "true": 1, "false": 0,
    }
    defines = {}
    enum_entries = {}
    roots = [repo / "src/main"]
    for root in roots:
        for path in sorted(root.rglob("*.h")):
            try:
                text = path.read_text(errors="ignore")
            except OSError:
                continue
            for raw in text.splitlines():
                m = DEFINE_RE.match(raw)
                if m:
                    name, value = m.group(1), m.group(2).strip()
                    defines.setdefault(name, value)
            # enum-элементы (Name = value | Name ,)
            stripped = _strip_c_comments(text)
            for block in re.findall(r"enum\s*\{([^{}]*)\}", stripped):
                nxt = 0
                for line in block.split(","):
                    line = line.strip()
                    if not line or "{" in line:
                        continue
                    m = ENUM_ENTRY_RE.match(line)
                    if not m:
                        continue
                    name, value = m.group(1), m.group(2)
                    if value is not None:
                        value = value.strip()
                        enum_entries.setdefault(name, value)
                        try:
                            nxt = int(value, 0) + 1
                        except ValueError:
                            pass
                    else:
                        enum_entries.setdefault(name, str(nxt))
                        nxt += 1

    def resolve(expr: str, depth=0):
        if expr is None or depth > 12:
            return None
        expr = expr.strip().rstrip(",")
        # убрать приведения типов: (uint16_t)x -> x
        expr = re.sub(
            r"\(\s*(?:u?int\d+_t|float|double|char|unsigned(?:\s+\w+)?|signed(?:\s+\w+)?|long(?:\s+\w+)?|short(?:\s+\w+)?)\s*\)",
            "",
            expr,
        )
        if _INT_RE.match(expr):
            return int(expr)
        if _HEX_RE.match(expr):
            return int(expr, 16)
        if _FLOAT_RE.match(expr):
            return float(expr)
        # простое выражение с именами (без вызовов функций)
        if re.match(r"^[\w\s+\-*/%<>()~|&^]+$", expr) and not re.search(r"\w\s*\(", expr):
            py = expr
            for name in set(re.findall(r"\b[A-Za-z_]\w*\b", expr)):
                if name in ("u", "U", "L", "f", "F"):
                    continue
                val = None
                if name in consts:
                    val = consts[name]
                elif name in defines:
                    val = resolve(defines[name], depth + 1)
                elif name in enum_entries:
                    val = resolve(enum_entries[name], depth + 1)
                if val is None:
                    return None
                py = re.sub(rf"\b{name}\b", str(val), py)
            try:
                return int(eval(py, {"__builtins__": {}}))
            except Exception:
                try:
                    return float(eval(py, {"__builtins__": {}}))
                except Exception:
                    return None
        return None

    # многопроходное разрешение #define и enum
    changed = True
    passes = 0
    while changed and passes < 8:
        changed = False
        passes += 1
        for name, value in list(defines.items()):
            if name in consts:
                continue
            v = resolve(value)
            if v is not None:
                consts[name] = v
                changed = True
        for name, value in list(enum_entries.items()):
            if name in consts:
                continue
            v = resolve(value)
            if v is not None:
                consts[name] = v
                changed = True
    consts["_defines"] = defines
    consts["_enums"] = enum_entries
    return consts


def resolve_const(name, consts, stack=()):
    if name is None:
        return 0
    if isinstance(name, (int, float)):
        return name
    if name in consts and not name.startswith("_"):
        return consts[name]
    if name in stack:
        return None
    expr = consts.get("_defines", {}).get(name) or consts.get("_enums", {}).get(name)
    if expr is None:
        return None
    if re.search(r"\w\s*\(", expr):
        return None
    # вычислить выражение
    py = expr
    for sub in set(re.findall(r"\b[A-Za-z_]\w*\b", expr)):
        if sub in ("u", "U", "L", "f", "F"):
            continue
        v = resolve_const(sub, consts, (*stack, name))
        if v is None:
            return None
        py = re.sub(rf"\b{sub}\b", str(v), py)
    try:
        return int(eval(py, {"__builtins__": {}}))
    except Exception:
        return None


# --------------------------------------------------------------------------------------
# определение типов полей через gcc (трюк из src/utils/settings.rb)
# --------------------------------------------------------------------------------------

TYPE_MAP = [
    (re.compile(r"\bbool\b"), "bool"),
    (re.compile(r"\bint8_t\b"), "int8_t"),
    (re.compile(r"\buint8_t\b"), "uint8_t"),
    (re.compile(r"\bint16_t\b"), "int16_t"),
    (re.compile(r"\buint16_t\b"), "uint16_t"),
    (re.compile(r"\bint32_t\b"), "int32_t"),
    (re.compile(r"\buint32_t\b"), "uint32_t"),
    (re.compile(r"\bfloat\b"), "float"),
    (re.compile(r"\bdouble\b"), "float"),
]

GCC_ERR_RE = re.compile(
    r"request for member ['‘]__type_detect_['’] in ['‘](var_\d+)\.[^'‘’]*['’], "
    r"which is of non-class type ['‘]([^'‘’]+)['’]"
)
CLANG_ERR_RE = re.compile(
    r"member reference base type ['‘]([^'‘’]+)['’] is not a structure or union.*?in ['‘](var_\d+)['’]",
    re.S,
)


def _classify(c_type: str):
    arr = re.search(r"char\s*\[(\d+)\]", c_type)
    if arr:
        return ("string", int(arr.group(1)) - 1)
    arr = re.search(r"\[(\d+)\]", c_type)
    if arr and "char" in c_type:
        return ("string", int(arr.group(1)) - 1)
    for rx, name in TYPE_MAP:
        if rx.search(c_type):
            return (name, None)
    return (None, None)


def probe_field_types(repo: Path, group_type: str, headers: list, fields: dict, defines=()):
    """fields: {var_placeholder_name: field_path}; возвращает {name: (type, str_max)}."""
    if not fields:
        return {}
    with tempfile.TemporaryDirectory(prefix="inavemu_probe_") as td:
        tdir = Path(td)
        # заглушки для архитектурных заголовков
        (tdir / "arm_math.h").write_text(
            "#pragma once\n"
            "#include <stdint.h>\n"
            "typedef float float32_t;\n"
            "typedef double float64_t;\n"
            "typedef struct { float32_t v[4]; } float32x4_t;\n"
        )
        (tdir / "DMA_TypeDef").unlink(missing_ok=True)
        shim = (
            "#pragma once\n"
            "typedef struct DMA_TypeDef_s DMA_TypeDef;\n"
            "typedef struct ADC_TypeDef_s ADC_TypeDef;\n"
            "typedef struct USART_TypeDef_s USART_TypeDef;\n"
        )
        (tdir / "shim_types.h").write_text(shim)
        prog = ["#include <stdint.h>", "#include <stdbool.h>", "#include <stddef.h>",
                '#include "shim_types.h"', '#include "platform.h"']
        for h in headers:
            prog.append(f'#include "{h}"')
        prog.append("int main() {")
        names = {}
        for i, (name, field) in enumerate(fields.items()):
            var = f"var_{i}"
            names[var] = name
            prog.append(f"{group_type} {var}; {var}.{field}.__type_detect_;")
        prog.append("return 0; }")
        (tdir / "probe.cpp").write_text("\n".join(prog) + "\n")
        cmd = [
            "gcc", "-fsyntax-only", "-std=c++11",
            "-I", str(tdir),
            "-I", str(repo / "src/test/unit"),
            "-I", str(repo / "src/main"),
        ]
        for d in defines:
            cmd.append(f"-D{d}")
        cmd.append(str(tdir / "probe.cpp"))
        out = subprocess.run(cmd, capture_output=True, text=True)
        stderr = out.stderr
        result = {}
        for m in GCC_ERR_RE.finditer(stderr):
            var, ctype = m.group(1), m.group(2)
            if var in names:
                t = _classify(ctype)
                if t[0]:
                    result[names[var]] = t
        for m in CLANG_ERR_RE.finditer(stderr):
            ctype, var = m.group(1), m.group(2)
            if var in names and names[var] not in result:
                t = _classify(ctype)
                if t[0]:
                    result[names[var]] = t
        return result


# --------------------------------------------------------------------------------------
# сбор данных
# --------------------------------------------------------------------------------------

TYPE_TO_CODE = {
    "bool": "uint8", "uint8_t": "uint8", "int8_t": "int8",
    "uint16_t": "uint16", "int16_t": "int16",
    "uint32_t": "uint32", "int32_t": "uint32",
    "float": "float", "string": "string",
}
TYPE_ORDER = ["uint8", "int8", "uint16", "int16", "uint32", "float", "string"]

# default_value: :target — константы generic-таргета (заголовки прошивки):
# battery.h: #ifndef VBAT_SCALE_DEFAULT → 1100; CURRENT_METER_SCALE/OFFSET —
# типовые 400 (40 мВ/А, ACS756) и 0. Таргет может переопределять.
TARGET_DEFAULTS = {
    "vbat_scale": 1100,
    "current_meter_scale": 400,
    "current_meter_offset": 0,
}


def build_settings(repo: Path, cfg: dict, consts: dict):
    tables = {}
    for t in cfg.get("tables") or []:
        tables[t["name"]] = t.get("values") or []
    if "off_on" not in tables:
        tables["off_on"] = ["OFF", "ON"]

    yaml_constants = cfg.get("constants") or {}
    for k, v in yaml_constants.items():
        consts.setdefault(k, v)

    settings = []
    warnings = []
    # все condition: из YAML — включаем как -D при gcc-пробах (полная прошивка)
    cond_defines = set()
    all_headers = []
    seen_headers = set()
    for group in cfg.get("groups") or []:
        if group.get("condition"):
            cond_defines.add(str(group["condition"]))
        for h in group.get("headers") or []:
            if h not in seen_headers:
                seen_headers.add(h)
                all_headers.append(h)
        for mem in group.get("members") or []:
            if mem.get("condition"):
                cond_defines.add(str(mem["condition"]))
    for group in cfg.get("groups") or []:
        group_type = group.get("type")
        pg_name = group.get("name")
        pg_id = resolve_const(pg_name, consts)
        section = group.get("value_type") or "MASTER_VALUE"
        members = group.get("members") or []

        # что нужно дотыкать gcc'ем
        need_probe = {}
        for mem in members:
            if not mem.get("type"):
                need_probe[mem["name"]] = mem.get("field") or mem["name"]
        probed = {}
        if need_probe:
            probed = probe_field_types(
                repo, group_type, all_headers, need_probe, defines=sorted(cond_defines)
            )
            # второй проход: по одной пробе на поле — для полей, «съеденных» фатальными ошибками
            missing = {k: v for k, v in need_probe.items() if k not in probed}
            for name, field in missing.items():
                one = probe_field_types(
                    repo, group_type, all_headers, {name: field}, defines=sorted(cond_defines)
                )
                probed.update(one)

        for mem in members:
            name = mem["name"]
            table = mem.get("table")
            declared = mem.get("type")
            if declared == "bool" and not table:
                table = "off_on"
            str_max = None
            if declared:
                typ = TYPE_TO_CODE.get(declared)
                if typ is None:
                    warnings.append(f"{name}: неизвестный тип {declared!r}")
                    continue
            else:
                if name in probed:
                    typ, str_max = probed[name]
                    if typ == "bool" and not table:
                        table = "off_on"
                else:
                    warnings.append(f"{name}: тип не определён (group {pg_name})")
                    continue
            values = tables.get(table) if table else None
            if table and values is None:
                warnings.append(f"{name}: нет таблицы {table!r}")
                continue

            entry = {
                "name": name,
                "type": TYPE_TO_CODE.get(typ, typ),
                "section": section,
                "pg": pg_id if pg_id is not None else 0,
                "pg_name": pg_name,
            }
            if values is not None:
                entry["mode"] = 64  # MODE_LOOKUP
                entry["values"] = list(values)
                entry["min"] = 0
                entry["max"] = len(values) - 1
            else:
                entry["mode"] = 0  # MODE_DIRECT
                if typ == "string":
                    mx = mem.get("max")
                    if mx is not None:
                        entry["min"] = 0
                        entry["max"] = int(resolve_const(mx, consts) or 0)
                    elif str_max is not None:
                        entry["min"] = 0
                        entry["max"] = str_max
                    else:
                        warnings.append(f"{name}: нет max для строки")
                        entry["min"], entry["max"] = 0, 0
                else:
                    mn = mem.get("min", 0)
                    mx = mem.get("max")
                    entry["min"] = int(resolve_const(mn, consts) or 0) if mn is not None else 0
                    if mx is None:
                        warnings.append(f"{name}: нет max")
                        entry["max"] = 0
                    else:
                        rv = resolve_const(mx, consts)
                        entry["max"] = int(rv) if rv is not None else 0

            # значение по умолчанию
            dv = mem.get("default_value")
            if isinstance(dv, bool):
                entry["default"] = 1 if dv else 0
            elif isinstance(dv, (int, float)):
                entry["default"] = dv
            elif isinstance(dv, str):
                dv_name = dv[1:] if dv.startswith(":") else dv
                if dv_name in ("zero", "target", "target specific") or dv == "":
                    # :target — дефолт задаёт target.h; для generic-таргета
                    # известные константы (battery.h: #ifndef VBAT_SCALE_DEFAULT ...)
                    entry["default"] = TARGET_DEFAULTS.get(
                        name, "" if typ == "string" else 0)
                elif values is not None and dv_name in values:
                    entry["default"] = values.index(dv_name)
                elif values is not None and dv in values:
                    entry["default"] = values.index(dv)
                else:
                    rv = resolve_const(dv_name, consts)
                    if rv is not None:
                        entry["default"] = rv
                    elif typ == "string":
                        entry["default"] = dv
                    else:
                        warnings.append(f"{name}: default {dv!r} не разрешён")
                        entry["default"] = 0
            else:
                entry["default"] = "" if typ == "string" else 0
            settings.append(entry)
    return settings, tables, warnings


def extract_osd_layout_defaults(repo: Path):
    """Дефолтная раскладка OSD (layout 0) из pgResetFn_osdLayoutsConfig (io/osd.c).

    Позиции элементов не лежат в settings.yaml — их задаёт сброс PG в osd.c:
    `item_pos[0][OSD_X] = OSD_POS(x, y) [| OSD_VISIBLE_FLAG]`. Порядок id —
    enum osd_items_e из io/osd.h. Раскладки 1..3 сбросом не трогаются (нули).
    """
    osd_h = (repo / "src/main/io/osd.h").read_text(errors="ignore")
    osd_c = (repo / "src/main/io/osd.c").read_text(errors="ignore")

    m = re.search(r"typedef enum\s*\{(.*?)\}\s*osd_items_e;", osd_h, re.S)
    if not m:
        sys.exit("[gen] не найден enum osd_items_e в io/osd.h")
    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)
    body = re.sub(r"//[^\n]*", "", body)
    names = []
    for part in body.split(","):
        mm = re.search(r"(OSD_[A-Z0-9_]+)", part.strip())
        if mm:
            names.append(mm.group(1))
    if not names or names[-1] != "OSD_ITEM_COUNT":
        sys.exit("[gen] enum osd_items_e не заканчивается OSD_ITEM_COUNT")
    names = names[:-1]  # OSD_ITEM_COUNT — счётчик, не элемент
    ids = {n: i for i, n in enumerate(names)}

    fn = osd_c[osd_c.index("pgResetFn_osdLayoutsConfig"):]
    fn = fn[: fn.index("\n}", fn.index("{"))]
    pat = re.compile(
        r"item_pos\[0\]\[(OSD_[A-Z0-9_]+)\]\s*=\s*OSD_POS\(\s*(-?\d+)\s*,\s*(-?\d+)\s*\)"
        r"\s*(\|\s*OSD_VISIBLE_FLAG)?\s*;")
    defaults = [0] * len(names)
    for name, x, y, vis in pat.findall(fn):
        if name not in ids:
            continue
        val = (int(x) & 0x3F) | ((int(y) & 0x3F) << 6)
        if vis:
            val |= 0x2000
        defaults[ids[name]] = val
    print(f"[gen] OSD: элементов {len(names)}, "
          f"видимых по умолчанию {sum(1 for v in defaults if v & 0x2000)}")
    return defaults


def extract_boxes(repo: Path):
    text = (repo / "src/main/fc/fc_msp_box.c").read_text(errors="ignore")
    boxes = []
    for m in re.finditer(
        r'\{\s*\.boxId\s*=\s*(\w+)\s*,\s*\.boxName\s*=\s*"([^"]*)"\s*,\s*\.permanentId\s*=\s*(\d+)\s*\}',
        text,
    ):
        boxes.append({"box": m.group(1), "name": m.group(2), "permanent_id": int(m.group(3))})
    return boxes


def extract_features(configurator: Path | None):
    if not configurator:
        return []
    fc_js = configurator / "js/fc.js"
    text = fc_js.read_text(errors="ignore")
    m = re.search(r"getFeatures: function \(\) \{.*?var features = \[(.*?)\];", text, re.S)
    if not m:
        return []
    features = []
    for fm in re.finditer(r"\{bit:\s*(\d+).*?name:\s*'([^']+)'", m.group(1)):
        features.append({"bit": int(fm.group(1)), "name": fm.group(2)})
    for fm in re.finditer(r'\{bit:\s*(\d+).*?name:\s*"([^"]+)"', m.group(1)):
        features.append({"bit": int(fm.group(1)), "name": fm.group(2)})
    features.sort(key=lambda f: f["bit"])
    return features


def extract_serial_functions(configurator: Path | None):
    if not configurator:
        return []
    text = (configurator / "js/serialPortHelper.js").read_text(errors="ignore")
    m = re.search(r"functionIDs = \{(.*?)\};", text, re.S)
    if not m:
        return []
    funcs = []
    for fm in re.finditer(r"'(\w+)':\s*(\d+)", m.group(1)):
        funcs.append({"name": fm.group(1), "bit": int(fm.group(2))})
    funcs.sort(key=lambda f: f["bit"])
    ports = {}
    pm = re.search(r"identifierToName = \{(.*?)\};", text, re.S)
    if pm:
        for em in re.finditer(r"(\d+):\s*'([^']+)'", pm.group(1)):
            ports[int(em.group(1))] = em.group(2)
    return {"functions": funcs, "port_identifiers": ports}


def main():
    ap = argparse.ArgumentParser(description="Генератор данных inavemu из исходников INAV")
    ap.add_argument("inav", help="каталог с исходниками INAV (с src/main)")
    ap.add_argument("--configurator", help="каталог с исходниками INAV Configurator (необязательно)")
    ap.add_argument("-o", "--output", default=None, help="куда писать JSON")
    ap.add_argument("--tag", default=None, help="тег релиза (для поля source)")
    args = ap.parse_args()

    repo = Path(args.inav).resolve()
    settings_yaml = repo / "src/main/fc/settings.yaml"
    if not settings_yaml.exists():
        sys.exit(f"не найден {settings_yaml}")

    configurator = Path(args.configurator).resolve() if args.configurator else None

    print(f"[gen] разбираю {settings_yaml} ...")
    cfg = parse_yaml(settings_yaml.read_text(errors="ignore"))
    print(f"[gen] таблиц: {len(cfg.get('tables') or [])}, групп: {len(cfg.get('groups') or [])}")

    print("[gen] собираю константы из заголовков ...")
    consts = collect_constants(repo)

    print("[gen] строю таблицу настроек (gcc-пробы типов) ...")
    settings, tables, warnings = build_settings(repo, cfg, consts)

    boxes = extract_boxes(repo)
    features = extract_features(configurator)
    serial = extract_serial_functions(configurator)
    osd_layout_defaults = extract_osd_layout_defaults(repo)

    data = {
        "source": {
            "repo": "iNavFlight/inav",
            "tag": args.tag or "unknown",
            "api_version": "2.5.0",
            "settings_yaml": "src/main/fc/settings.yaml",
        },
        "settings": settings,
        "lookup_tables": {k: v for k, v in tables.items()},
        "features": features,
        "boxes": boxes,
        "serial": serial if isinstance(serial, dict) else {"functions": [], "port_identifiers": {}},
        "osd_layout_defaults": osd_layout_defaults,
    }

    out = Path(args.output) if args.output else (
        Path(__file__).resolve().parent.parent / "inavemu/data/inav_data.json"
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, ensure_ascii=False, indent=1))

    print(f"[gen] настроек: {len(settings)}, боксов: {len(boxes)}, "
          f"фич: {len(features)}, lookup-таблиц: {len(tables)}")
    print(f"[gen] записано: {out}")
    if warnings:
        print(f"[gen] предупреждений: {len(warnings)}")
        for w in warnings[:40]:
            print(f"  ! {w}")
        if len(warnings) > 40:
            print(f"  ... и ещё {len(warnings) - 40}")


if __name__ == "__main__":
    main()
