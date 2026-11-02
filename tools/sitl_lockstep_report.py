"""Создать локальный отчёт по парным трассам lockstep SITL, без сети и CDN."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bfemu.sitl_pair import analyze
from bfemu.sitl_pair_report import render



def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--html', type=Path, required=True, help='новый HTML-файл отчёта')
    parser.add_argument('--json', type=Path, help='новый JSON-файл с метриками и всеми отсчётами')
    args = parser.parse_args(argv)
    try:
        result = analyze(args.directory)
        # Exclusive creation prevents accidental overwrite of earlier evidence.
        if args.json and args.json.exists():
            raise FileExistsError(f'Файл уже существует: {args.json}')
        if args.html.exists():
            raise FileExistsError(f'Файл уже существует: {args.html}')
        with args.html.open('x', encoding='utf-8') as stream:
            stream.write(render(result))
        if args.json:
            with args.json.open('x', encoding='utf-8') as stream:
                json.dump(result, stream, ensure_ascii=False, allow_nan=False)
        print(result['reason'])
        return 0 if result['status'] == 'reproducible' else 2
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f'Отчёт не создан: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
