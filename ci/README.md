# Непрерывная интеграция

Готовый сценарий GitHub Actions лежит здесь как `github-actions-ci.yml`, а не в
`.github/workflows/`, по технической причине: токен, которым работает агент, не имеет права
`workflows` — GitHub отклоняет любой push, создающий или меняющий файлы рабочих процессов
(`refusing to allow a GitHub App to create or update workflow ... without workflows permission`).

## Как включить (одна команда)

```bash
mkdir -p .github/workflows
git mv ci/github-actions-ci.yml .github/workflows/ci.yml
git commit -m "ci: включить GitHub Actions"
git push
```

Либо через веб-интерфейс: **Actions → New workflow → set up a workflow yourself**, вставить
содержимое файла и сохранить.

## Что проверяется

| Задача | Что делает | Зачем |
|---|---|---|
| `lint` | `ruff check .` | замечаний сейчас ноль — любое новое видно сразу |
| `unit` | `pytest tests -q` на Python 3.10 / 3.11 / 3.12 | 39 быстрых проверок, ~10 секунд |
| `smoke` | `python tests/smoke_test.py` | 617 сквозных проверок протокола, CLI и вкладок |
| `windows` | проверка `.bat` (CRLF + только ASCII) на `windows-latest` + быстрые тесты | именно этот дефект ломал проект сразу после клонирования |
| `package` | `python -m build`, установка колеса, живой ответ `MSP_FC_VARIANT = BTFL` | гарантия, что `pip install` даёт рабочую команду `bfemu` |

Локально то же самое:

```bash
pip install -e ".[dev]"
ruff check .
pytest tests -q
python tests/smoke_test.py
```
