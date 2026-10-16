#!/usr/bin/env python3
"""Сохранить шаг работы одной командой: добавить изменения, закоммитить, отправить в origin.

    python3 tools/checkpoint.py "часть 5: что сделано на этом шаге"

Зачем отдельная программа, а не три команды git руками: работа идёт в песочнице, которая
может пересобраться в любой момент. Пока шаг не в `origin`, его можно потерять вместе с
песочницей — поэтому шаг закрывается одной командой, в которой невозможно забыть `push`.

Что делает по порядку:

1. `git add -A` — берёт всё, что не отфильтровано `.gitignore` (мусор запусков там уже есть);
2. `git commit -m <сообщение>` — если сохранять нечего, коммита не будет и программа честно
   об этом скажет (пустые коммиты только мешают читать историю);
3. `git push` — при первой отправке ветки сам выставит upstream (`-u origin`).

Отказы вместо риска: ветка `main`/`master` и «отсоединённый HEAD» — не сохраняются: рабочая
сессия ведётся в ветке `arena/*`, а в `main` попадает только через pull request.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PROTECTED = ("main", "master")


def git(*args: str, quiet: bool = False) -> subprocess.CompletedProcess:
    """Запустить git и вернуть результат, не роняя программу на ненулевом коде."""
    return subprocess.run(
        ["git", *args],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
        encoding="utf-8",
        errors="replace",
    )


def fail(message: str, result: subprocess.CompletedProcess | None = None) -> int:
    print(f"ОШИБКА: {message}", file=sys.stderr)
    if result is not None:
        for stream in (result.stdout, result.stderr):
            text = (stream or "").strip()
            if text:
                print(text, file=sys.stderr)
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Коммит и push текущего шага работы.")
    parser.add_argument("message", help="сообщение коммита: что именно сделано на шаге")
    parser.add_argument("--dry-run", action="store_true", help="показать, что было бы сохранено")
    args = parser.parse_args()

    if not args.message.strip():
        return fail("сообщение коммита пустое")

    branch = git("rev-parse", "--abbrev-ref", "HEAD")
    if branch.returncode != 0:
        return fail("не удалось определить ветку (это вообще репозиторий git?)", branch)
    name = branch.stdout.strip()
    if name in PROTECTED:
        return fail(f"ветка {name} защищена: работа сохраняется в ветке arena/*, а в main — через pull request")
    if name == "HEAD":
        return fail("отсоединённый HEAD: сначала переключитесь на рабочую ветку")

    added = git("add", "-A")
    if added.returncode != 0:
        return fail("не удалось добавить изменения в индекс", added)

    staged = git("diff", "--cached", "--name-status")
    if staged.returncode != 0:
        return fail("не удалось посмотреть, что попадёт в коммит", staged)
    if not staged.stdout.strip():
        print(f"В ветке {name} сохранять нечего: рабочее дерево совпадает с последним коммитом.")
        pushed = git("rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{upstream}")
        if pushed.returncode != 0:
            print("У ветки нет upstream — при первом изменении программа сама отправит её в origin.")
        return 0

    if args.dry_run:
        print(f"Ветка {name}, было бы сохранено:")
        print(staged.stdout.rstrip())
        return 0

    print(f"Ветка {name}, файлов в коммите: {len(staged.stdout.splitlines())}")
    for line in staged.stdout.splitlines():
        print(f"  {line}")

    commit = git("commit", "-m", args.message)
    if commit.returncode != 0:
        return fail("коммит не создан", commit)
    print(commit.stdout.strip().splitlines()[0] if commit.stdout.strip() else "коммит создан")

    upstream = git("rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{upstream}")
    push = git("push") if upstream.returncode == 0 else git("push", "-u", "origin", name)
    if push.returncode != 0:
        return fail("не удалось отправить ветку в origin", push)
    print(push.stderr.strip() or push.stdout.strip())

    head = git("rev-parse", "--short", "HEAD").stdout.strip()
    print(f"Готово: шаг сохранён в origin ({name}, коммит {head}).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
