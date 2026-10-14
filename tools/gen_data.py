#!/usr/bin/env python3
"""
Генератор данных для виртуального полётного контроллера (bfemu).

Разбирает исходники Betaflight и вытаскивает:
  * таблицу настроек CLI (имя, тип, диапазон / список значений, секция dump, PG-группа);
  * таблицы lookup-значений (напр. `Allowed values: PT1, BIQUAD, ...`);
  * имена фич (feature), боксов (aux/modes), биперов, функций портов, типов MCU.

Использование:
    python3 tools/gen_data.py /path/to/betaflight [-o data/bf_data.json]

Ничего, кроме stdlib, не требуется.
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

# --------------------------------------------------------------------------------------
# примитивный препроцессор: разворачиваем #if/#ifdef, чтобы увидеть все ветки настроек
# --------------------------------------------------------------------------------------


def strip_inactive(lines, defines=None):
    """Возвращает список (lineno, text) только для строк, которые препроцессор не отбросил бы.

    Реализовано «мягко»: мы сознательно оставляем текст ВСЕХ веток #if/#else, потому что
    эмулятору нужен максимум настроек (лишние не мешают). Убираем только строки, целиком
    состоящие из директив.
    """
    out = []
    for i, raw in enumerate(lines):
        s = raw.strip()
        if s.startswith("#"):
            continue
        out.append((i, raw))
    return out


# --------------------------------------------------------------------------------------
# макросы (#define NAME value)
# --------------------------------------------------------------------------------------

DEFINE_RE = re.compile(r"^\s*#\s*define\s+(\w+)\s+(.+?)\s*(?://.*)?$")


def collect_defines(repo: Path):
    defines = {}
    roots = [
        repo / "src/main",
        repo / "src/platform",
    ]
    for root in roots:
        for path in root.rglob("*"):
            if path.suffix not in (".h", ".c"):
                continue
            try:
                text = path.read_text(errors="ignore")
            except OSError:
                continue
            for raw in text.splitlines():
                m = DEFINE_RE.match(raw)
                if not m:
                    continue
                name, value = m.group(1), m.group(2).strip()
                if name in defines:
                    continue
                defines[name] = value
    return defines


def resolve_int(expr: str, defines: dict, depth=0):
    """Пытается вычислить C-выражение из констант. Возвращает int или None."""
    if expr is None or depth > 8:
        return None
    expr = expr.strip().rstrip(",")
    expr = re.sub(r"/\*.*?\*/", "", expr)
    expr = expr.strip()
    if not expr:
        return None
    if re.fullmatch(r"-?(0x[0-9a-fA-F]+|\d+)", expr):
        return int(expr, 0)
    # подстановка макросов (одно слово)
    if re.fullmatch(r"\w+", expr):
        if expr in defines:
            return resolve_int(defines[expr], defines, depth + 1)
        return None
    # (A) / (A + B) / A | B / STR(x)
    inner = expr
    while inner.startswith("(") and inner.endswith(")"):
        # снимаем внешние скобки, только если они парные
        depth_paren = 0
        ok = True
        for i, ch in enumerate(inner):
            if ch == "(":
                depth_paren += 1
            elif ch == ")":
                depth_paren -= 1
                if depth_paren == 0 and i != len(inner) - 1:
                    ok = False
                    break
        if not ok:
            break
        inner = inner[1:-1].strip()
    # заменяем идентификаторы на значения
    def repl(m):
        name = m.group(0)
        val = resolve_int(defines.get(name, ""), defines, depth + 1) if name in defines else None
        return str(val) if val is not None else name

    substituted = re.sub(r"\b[A-Za-z_]\w*\b", repl, inner)
    if re.fullmatch(r"[0-9a-fA-FxX\s\+\-\*/<>\|&\(\)~]+", substituted):
        py_expr = substituted.replace("&&", " and ").replace("||", " or ")
        try:
            return int(eval(py_expr, {"__builtins__": {}}, {}))  # noqa: S307 - локальные константы
        except Exception:
            return None
    return None


# --------------------------------------------------------------------------------------
# lookup-таблицы из settings.c
# --------------------------------------------------------------------------------------

# объявление массива имён: «const char * const name[] = {», «const char* const name[] = {»,
# а также с явным размером — «const char * const debugModeNames[DEBUG_COUNT] = {»
TABLE_ARRAY_START_RE = re.compile(
    r"(?:static\s+)?const\s+char\s*\*+\s*const\s+(\w+)\s*\[\s*[^\]]*\]\s*=\s*\{"
)
TABLE_ENTRY_RE = re.compile(r'"([^"]*)"')


def parse_lookup_tables(settings_c: str):
    tables = {}
    lines = settings_c.splitlines()
    i = 0
    while i < len(lines):
        m = TABLE_ARRAY_START_RE.match(lines[i].strip())
        if not m:
            i += 1
            continue
        name = m.group(1)
        values = []
        j = i
        while j < len(lines):
            line = lines[j]
            if j > i and "};" in line:
                values.extend(TABLE_ENTRY_RE.findall(line))
                break
            if j > i:
                values.extend(TABLE_ENTRY_RE.findall(line))
            else:
                values.extend(TABLE_ENTRY_RE.findall(line.split("=", 1)[1]))
            j += 1
        tables[name] = [v for v in values if v != ""]
        i = j + 1
    return tables


def parse_lookup_table_index(settings_h: str, settings_c: str):
    """Таблица `lookupTables[]` позиционно соответствует enum lookupTableIndex_e."""
    enum_block = re.search(r"typedef enum\s*\{(.*?)\}\s*lookupTableIndex_e", settings_h, re.S)
    if not enum_block:
        return {}, {}
    enum_names = []
    for raw in enum_block.group(1).splitlines():
        s = raw.strip().rstrip(",")
        if not s or s.startswith("//"):
            continue
        m = re.match(r"(TABLE_\w+)", s)
        if m:
            enum_names.append(m.group(1))

    arr = re.search(r"const lookupTableEntry_t lookupTables\[\]\s*=\s*\{(.*?)\n\};", settings_c, re.S)
    entries = []
    if arr:
        for raw in arr.group(1).splitlines():
            s = raw.strip()
            m = re.match(r"\{\s*(\w+)\s*,", s) or re.match(r"LOOKUP_TABLE_ENTRY\(\s*(\w+)\s*\)", s)
            if m:
                entries.append(m.group(1))

    index_to_table = {}
    for idx, name in enumerate(enum_names):
        if idx < len(entries):
            index_to_table[name] = entries[idx]
    table_to_values = {}
    return index_to_table, table_to_values


# --------------------------------------------------------------------------------------
# основная таблица настроек (valueTable[])
# --------------------------------------------------------------------------------------

FLAG_TYPES = {
    "VAR_UINT8": ("uint8", 0, 255),
    "VAR_INT8": ("int8", -128, 127),
    "VAR_UINT16": ("uint16", 0, 65535),
    "VAR_INT16": ("int16", -32768, 32767),
    "VAR_UINT32": ("uint32", 0, 4294967295),
    "VAR_INT32": ("int32", -2147483648, 2147483647),
}
FLAG_SECTIONS = ["MASTER_VALUE", "PROFILE_VALUE", "PROFILE_RATE_VALUE", "HARDWARE_VALUE", "PROFILE_BATTERY_VALUE"]
FLAG_MODES = ["MODE_LOOKUP", "MODE_ARRAY", "MODE_BITSET", "MODE_STRING", "MODE_LOOKUP_IDENTIFIER", "MODE_DIRECT"]


def split_args(text: str):
    """Разбивает аргументы верхнего уровня (учитывая вложенные {} и ())."""
    parts, depth, cur = [], 0, []
    for ch in text:
        if ch in "{(":
            depth += 1
        elif ch in "})":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    if cur:
        parts.append("".join(cur).strip())
    return [p for p in parts if p]


def parse_value_table(settings_c: str, defines: dict, param_names: dict, tables: dict, table_index: dict, unresolved: dict | None = None):
    body = re.search(r"const clivalue_t valueTable\[\]\s*=\s*\{(.*?)\n\};", settings_c, re.S)
    if not body:
        raise SystemExit("не найден valueTable[] в settings.c")
    text = body.group(1)
    # склеиваем многострочные записи
    entries, cur, depth = [], [], 0
    for line in text.splitlines():
        line = re.sub(r"//.*$", "", line)
        if not line.strip():
            continue
        depth += line.count("{") - line.count("}")
        cur.append(line)
        if depth == 0 and cur:
            entries.append("\n".join(cur))
            cur = []
    if cur:
        entries.append("\n".join(cur))

    out = []
    for entry in entries:
        e = entry.strip()
        if not e.startswith("{") or not ("offsetof" in e or "PG_ARRAY_ELEMENT_OFFSET" in e):
            continue
        inner = e.strip().rstrip(",").strip()
        inner = inner[1:-1].strip()
        args = split_args(inner)
        if len(args) < 5:
            continue
        name_tok = args[0].strip()
        m = re.match(r'"(.*)"', name_tok)
        if m:
            name = m.group(1)
        else:
            name = param_names.get(name_tok, name_tok)
        flags_tok = args[1]
        cfg_tok = args[2]
        pg_tok = args[3].strip()

        vtype = "uint8"
        vmin, vmax = 0, 255
        for f, (t, lo, hi) in FLAG_TYPES.items():
            if re.search(rf"\b{f}\b", flags_tok):
                vtype, vmin, vmax = t, lo, hi
                break
        section = next((s for s in FLAG_SECTIONS if s in flags_tok), "MASTER_VALUE")
        mode = next((m for m in FLAG_MODES if m in flags_tok), "MODE_DIRECT")

        values = None
        if mode == "MODE_LOOKUP":
            tm = re.search(r"TABLE_(\w+)", cfg_tok)
            if tm:
                tbl_key = "TABLE_" + tm.group(1)
                tbl_name = table_index.get(tbl_key, "lookupTable" + tm.group(1).title().replace("_", ""))
                values = tables.get(tbl_name)
                if values is None:
                    # точное имя не нашлось — пробуем варианты написания, но без нечётких
                    # совпадений: они подставляли чужую таблицу (debug_mode получал ROLL/PITCH
                    # от таблицы acro_trainer_debug_axis)
                    base = tm.group(1)
                    candidates = (base, "lookupTable" + base, "lookup" + base)
                    for cand in candidates:
                        if cand in tables:
                            values = tables[cand]
                            break
                    if values is None:
                        # имена таблиц в коде отличаются написанием (TABLE_LEDSTRIP_COLOR →
                        # lookupLedstripColors) — ищем по нормализованному имени, но точному
                        normalized = {k.lower().replace("_", ""): v for k, v in tables.items()}
                        for cand in candidates:
                            for tail in ("", "s"):
                                hit = normalized.get((cand + tail).lower().replace("_", ""))
                                if hit is not None:
                                    values = hit
                                    break
                            if values is not None:
                                break
                if values is None and unresolved is not None:
                    unresolved.setdefault(tbl_key, []).append(name)
        else:
            mm = re.search(r"(minmaxUnsigned|minmax)\s*=\s*\{\s*([^,]+),\s*([^}]+)\}", cfg_tok)
            if mm:
                lo = resolve_int(mm.group(2), defines)
                hi = resolve_int(mm.group(3), defines)
                if lo is not None:
                    vmin = lo
                if hi is not None:
                    vmax = hi
            al = re.search(r"arraylength\s*=\s*\{\s*([^}]+)\}", cfg_tok)
            if al:
                n = resolve_int(al.group(1), defines)
                values = None
                vmin, vmax = 0, 65535
                if mode == "MODE_ARRAY":
                    vmax = (n or 1) - 1
        last = args[4] if len(args) > 4 else ""
        off = re.search(r"offsetof\(\s*(\w+)\s*,\s*(\w+)\s*\)", last)
        if not off:  # PG_ARRAY_ELEMENT_OFFSET(структура, индекс, поле)
            off = re.search(r"PG_ARRAY_ELEMENT_OFFSET\(\s*(\w+)\s*,\s*\d+\s*,\s*(\w+)\s*\)", last)
        struct_type = off.group(1) if off else None
        struct_field = off.group(2) if off else None

        out.append(
            {
                "name": name,
                "struct": struct_type,
                "field": struct_field,
                "type": vtype,
                "section": section,
                "mode": mode,
                "min": vmin,
                "max": vmax,
                "values": values,
                "pg": pg_tok,
            }
        )
    return out


# --------------------------------------------------------------------------------------
# списки фич / боксов / биперов / функций портов
# --------------------------------------------------------------------------------------


def parse_c_string_list(text: str, start_marker: str, stop_marker: str = "};"):
    m = re.search(re.escape(start_marker) + r"(.*?)" + re.escape(stop_marker), text, re.S)
    if not m:
        return None
    body = re.sub(r"//.*", "", m.group(1))
    return re.findall(r'"([^"]*)"', body)


def parse_feature_names(cli_c: str):
    """Имена фич по порядку битов (пустые позиции сохранены — они тоже занимают бит)."""
    names = parse_c_string_list(cli_c, "static const char * const featureNames[] = {")
    if not names:
        return None
    return names


def feature_bits_from_names(names):
    """{имя: бит} — позиция в featureNames[] и есть номер бита (см. features_e в feature.h)."""
    if not names:
        return None
    return {name: index for index, name in enumerate(names) if name}


def parse_serial_functions(cli_c: str):
    m = re.search(r"static const char \* const serialPortNames\[\]\s*=\s*\{(.*?)\};", cli_c, re.S)
    return re.findall(r'"([^"]*)"', m.group(1)) if m else None


def parse_box_names(repo: Path):
    """Имена боксов (aux modes) в порядке MSP_BOXNAMES."""
    for path in repo.rglob("rc_controls.h"):
        text = path.read_text(errors="ignore")
        names = parse_c_string_list(text, "BOXNAMES = {")
        if names:
            return [n for n in names if n]
    return None


def parse_box_ids(repo: Path):
    for path in repo.rglob("boxid_e*"):
        pass
    for path in repo.rglob("*.h"):
        text = path.read_text(errors="ignore")
        m = re.search(r"typedef enum\s*\{(.*?)\}\s*boxId_e", text, re.S)
        if m:
            ids = []
            for raw in m.group(1).splitlines():
                s = raw.strip().rstrip(",")
                if not s or s.startswith("//"):
                    continue
                mm = re.match(r"(BOX\w+)", s)
                if mm:
                    ids.append(mm.group(1))
            return ids
    return None


def parse_mcu_types(cli_c: str):
    m = re.search(r"static const char \*mcuTypeNames\[\]\s*=\s*\{(.*?)\};", cli_c, re.S)
    return re.findall(r'"([^"]*)"', m.group(1)) if m else None


def parse_param_names(repo: Path):
    out = {}
    for name in ("parameter_names.h",):
        for path in repo.rglob(name):
            for raw in path.read_text(errors="ignore").splitlines():
                m = re.match(r"\s*#\s*define\s+(PARAM_NAME_\w+)\s+\"([^\"]+)\"", raw)
                if m:
                    out[m.group(1)] = m.group(2)
    return out


# --------------------------------------------------------------------------------------
# имена PG-групп для секций dump
# --------------------------------------------------------------------------------------


def parse_pg_names(repo: Path):
    out = {}
    for path in repo.rglob("*.h"):
        try:
            text = path.read_text(errors="ignore")
        except OSError:
            continue
        m = re.search(r"typedef enum\s*\{(.*?)\}\s*pgn_e", text, re.S)
        if not m:
            continue
        for raw in m.group(1).splitlines():
            s = raw.strip().rstrip(",")
            mm = re.match(r"(PG_\w+)", s)
            if mm:
                out[mm.group(1)] = mm.group(1)
    return out


# --------------------------------------------------------------------------------------
# enum-константы (для разбора значений по умолчанию)
# --------------------------------------------------------------------------------------

ENUM_BLOCK_RE = re.compile(r"(?:typedef\s+)?enum(?:\s+\w+)?\s*\{(.*?)\}\s*(\w+)?\s*;", re.S)


def collect_enum_constants(repo: Path):
    """Возвращает {ИМЯ_КОНСТАНТЫ: int} по всем enum в src/main."""
    consts = {}
    for path in list(repo.glob("src/main/**/*.h")) + list(repo.glob("src/main/**/*.c")):
        try:
            text = path.read_text(errors="ignore")
        except OSError:
            continue
        for m in ENUM_BLOCK_RE.finditer(text):
            body = re.sub(r"//.*", "", m.group(1))
            next_value = 0
            for item in body.split(","):
                item = item.strip()
                if not item:
                    continue
                item = item.split("=")[0].strip() if "=" not in item else item
                if "=" in item:
                    name, _, expr = item.partition("=")
                    name = name.strip()
                    value = resolve_int(expr.strip(), {})
                    if value is None:
                        value = next_value
                else:
                    name = item.strip()
                    value = next_value
                if re.fullmatch(r"\w+", name):
                    if name not in consts:
                        consts[name] = value
                    next_value = value + 1
    return consts


# --------------------------------------------------------------------------------------
# заводские значения (pgResetFn_* / PG_RESET_TEMPLATE)
# --------------------------------------------------------------------------------------

ASSIGN_RE = re.compile(r"(?:(\w+)\s*->\s*(\w+)|\.\s*(\w+))\s*=\s*([^;,]+)[;,]")
RESET_FN_RE = re.compile(r"(?:void|STATIC_UNIT_TESTED void)\s+pgResetFn_(\w+)\s*\(\s*(\w+)\s*\*\s*(\w+)\s*\)\s*\{")
RESET_TPL_RE = re.compile(r"PG_RESET_TEMPLATE\s*\(\s*(\w+)\s*,\s*(\w+)\s*,")


def parse_defaults(repo: Path, defines: dict, enums: dict):
    """{(struct_type, field): value}. Берём и reset-функции, и PG_RESET_TEMPLATE."""
    defaults = {}
    files = list(repo.glob("src/main/**/*.c"))
    for path in files:
        try:
            text = path.read_text(errors="ignore")
        except OSError:
            continue
        # --- PG_RESET_TEMPLATE(Type, name, .field = value, ...) ---
        for m in RESET_TPL_RE.finditer(text):
            struct_type = m.group(1)
            args = text[m.end():]
            depth = 1
            buf = []
            for ch in args:
                if ch == "(":
                    depth += 1
                elif ch == ")":
                    depth -= 1
                    if depth == 0:
                        break
                buf.append(ch)
            for am in ASSIGN_RE.finditer("".join(buf)):
                field = am.group(3) or am.group(2)
                if not field:
                    continue
                val = resolve_value(am.group(4), defines, enums)
                if val is not None and (struct_type, field) not in defaults:
                    defaults[(struct_type, field)] = val
        # --- pgResetFn_x(Type *ptr) { ptr->field = value; } ---
        for m in RESET_FN_RE.finditer(text):
            struct_type = m.group(2)
            ptr = m.group(3)
            start = m.end()
            depth = 1
            i = start
            while i < len(text) and depth:
                if text[i] == "{":
                    depth += 1
                elif text[i] == "}":
                    depth -= 1
                i += 1
            body = text[start:i]
            for am in re.finditer(rf"{ptr}\s*->\s*(\w+)\s*=\s*([^;]+);", body):
                field = am.group(1)
                val = resolve_value(am.group(2), defines, enums)
                if val is not None and (struct_type, field) not in defaults:
                    defaults[(struct_type, field)] = val
    return defaults


def resolve_value(expr: str, defines: dict, enums: dict, depth=0):
    if depth > 6:
        return None
    expr = expr.strip()
    if not expr:
        return None
    low = expr.lower()
    if low in ("true", "false"):
        return 1 if low == "true" else 0
    if re.fullmatch(r"-?\d+", expr):
        return int(expr)
    if re.fullmatch(r"0x[0-9a-fA-F]+", expr):
        return int(expr, 16)
    if re.fullmatch(r"\w+", expr):
        if expr in enums:
            return enums[expr]
        if expr in defines:
            v = resolve_int(defines[expr], defines)
            if v is not None:
                return v
            return resolve_value(defines[expr], defines, enums, depth + 1)
        return None
    v = resolve_int(expr, defines)
    if v is not None:
        return v
    # выражение с enum-константами: (GYRO_CONFIG_USE_GYRO_1 + 1)
    sub = expr
    for name, val in enums.items():
        sub = re.sub(rf"\b{name}\b", str(val), sub)
    v = resolve_int(sub, defines)
    return v


def parse_boxes(repo: Path):
    boxes = []
    for path in repo.rglob("msp_box.c"):
        text = path.read_text(errors="ignore")
        for raw in text.splitlines():
            s = raw.strip()
            if s.startswith("//"):
                continue
            m = re.search(r'\.boxId\s*=\s*(\w+)\s*,\s*\.boxName\s*=\s*"([^"]+)"\s*,\s*\.permanentId\s*=\s*(\d+)', s)
            if m:
                boxes.append({"box_id": m.group(1), "name": m.group(2), "permanent_id": int(m.group(3))})
    return boxes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("repo", type=Path, help="путь к чекауту betaflight")
    ap.add_argument("-o", "--out", type=Path, default=None)
    args = ap.parse_args()
    repo: Path = args.repo

    settings_c_path = repo / "src/main/cli/settings.c"
    settings_h_path = repo / "src/main/cli/settings.h"
    cli_c_path = repo / "src/main/cli/cli.c"
    msp_protocol = (repo / "src/main/msp/msp_protocol.h").read_text(errors="ignore")
    version_h = (repo / "src/main/build/version.h").read_text(errors="ignore")

    settings_c = settings_c_path.read_text(errors="ignore")
    settings_h = settings_h_path.read_text(errors="ignore")
    cli_c = cli_c_path.read_text(errors="ignore")

    defines = collect_defines(repo)
    param_names = parse_param_names(repo)
    # lookup-таблицы лежат не только в settings.c: debugModeNames — в build/debug.c,
    # currentMeterSourceNames — в sensors/current.c и т.д. Сканируем весь src/main.
    all_sources = "\n".join(
        path.read_text(errors="ignore") for path in repo.glob("src/main/**/*.c")
    )
    tables = parse_lookup_tables(all_sources)
    table_index, _ = parse_lookup_table_index(settings_h, settings_c)
    unresolved: dict = {}
    value_table = parse_value_table(settings_c, defines, param_names, tables, table_index, unresolved)
    enums = collect_enum_constants(repo)
    defaults = parse_defaults(repo, defines, enums)
    # Точечные поправки: парсер берёт первое присваивание в reset-функции, а у motorConfig
    # сначала идёт ветка BRUSHED_MOTORS (minthrottle 1000), потом штатная (1070).
    # У failsafeConfig_t в failsafe.c сначала идёт ветка USE_RACE_PRO
    # (DEFAULT_FAILSAFE_RECOVERY_DELAY 1), затем штатная (5, «500 ms of valid rx data»),
    # а stick_threshold вообще задан только в PG_RESET_TEMPLATE — берём штатные значения.
    for key, value in {("motorConfig_t", "minthrottle"): 1070,
                       ("failsafeConfig_t", "failsafe_recovery_delay"): 5,
                       ("failsafeConfig_t", "failsafe_stick_threshold"): 30}.items():
        defaults[key] = value
    hits = 0
    for st in value_table:
        key = (st.get("struct"), st.get("field"))
        if key in defaults:
            st["default"] = defaults[key]
            hits += 1
        else:
            st["default"] = None
    print(f"заводских значений найдено: {hits}/{len(value_table)} (enum-констант: {len(enums)})")

    def const(name, default=None):
        m = re.search(rf"#define\s+{name}\s+([0-9]+)", msp_protocol + version_h)
        return int(m.group(1)) if m else default

    fc_major = None
    m = re.search(r"#define\s+FC_VERSION_MAJOR\s+([0-9]+)", version_h)
    if m:
        fc_major = int(m.group(1))

    data = {
        "source": {
            "repo": "betaflight/betaflight",
            "api_version": f"{const('API_VERSION_MAJOR', 1)}.{const('API_VERSION_MINOR', 46)}.0",
            "msp_protocol_version": 0,
            "fc_version": fc_major if fc_major is not None else None,
        },
        "settings": value_table,
        "lookup_tables": tables,
        "features": [name for name in (parse_feature_names(cli_c) or []) if name],
        # номер бита задаёт позиция в featureNames[] — с пропусками, как в feature.h
        "feature_bits": feature_bits_from_names(parse_feature_names(cli_c)),
        "boxes": parse_boxes(repo) or parse_box_names(repo),
        "box_ids": parse_box_ids(repo),
        "serial_functions": parse_serial_functions(cli_c),
        "mcu_types": parse_mcu_types(cli_c),
    }
    out = args.out or Path(__file__).resolve().parent.parent / "data" / "bf_data.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=1, ensure_ascii=False))
    lookup_ok = sum(1 for s in value_table if s["mode"] == "MODE_LOOKUP" and s["values"])
    lookup_all = sum(1 for s in value_table if s["mode"] == "MODE_LOOKUP")
    print(f"настроек: {len(value_table)} (lookup с именами: {lookup_ok}/{lookup_all})")
    if unresolved:
        print("НЕ РАЗОБРАНЫ lookup-таблицы:")
        for tbl_key, names in sorted(unresolved.items()):
            print(f"   {tbl_key}: {', '.join(sorted(set(names))[:6])}")
    print(f"lookup-таблиц: {len(tables)}, фич: {len(data['features'] or [])}, боксов: {len(data['boxes'] or [])}")
    print("биты фич:", data.get("feature_bits"))
    print(f"api: {data['source']['api_version']}, mcu: {data['mcu_types']}")
    print(f"-> {out}")


if __name__ == "__main__":
    main()
