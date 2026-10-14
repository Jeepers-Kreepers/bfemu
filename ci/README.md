# Непрерывная интеграция

Этот файл — **эталон сценария**. Рабочая копия живёт в `.github/workflows/ci.yml` и правится
только вручную: токен, которым работает агент, не имеет права `workflows`, поэтому GitHub
отклоняет любой push, создающий или меняющий файлы рабочих процессов
(`refusing to allow a GitHub App to create or update workflow ... without workflows permission`).

## Как включить или обновить

Если рабочего процесса ещё нет:

```bash
mkdir -p .github/workflows
cp ci/github-actions-ci.yml .github/workflows/ci.yml
git add .github/workflows/ci.yml && git commit -m "ci: включить GitHub Actions" && git push
```

Если он уже есть — обновить из эталона той же командой `cp` (либо через веб-интерфейс:
открыть `.github/workflows/ci.yml` → карандаш → вставить содержимое `ci/github-actions-ci.yml`).
**Держите файлы одинаковыми**: расхождение заметно не сразу, зато красным в чужом pull request.

## Грабли, на которые уже наступили

* **Русский текст и Windows.** На `windows-latest` Python пишет stdout в кодировке системной
  кодовой страницы (cp1252), и первое же русское слово роняет шаг с `UnicodeEncodeError` —
  причём независимо от того, что проверял шаг. Лечится переменными `PYTHONUTF8: "1"` и
  `PYTHONIOENCODING: "utf-8"` на уровне всего сценария (они уже прописаны).
* **Фоновый процесс в шаге.** `bfemu ... &` без отвода вывода в файл держит шаг до таймаута:
  раннер ждёт закрытия потоков. Нужны `> log 2>&1 &` и `trap 'kill $PID' EXIT`.
* **Версии экшенов.** `actions/checkout@v4` и `actions/setup-python@v5` работают на Node 20
  и дают предупреждение об устаревании — в эталоне стоят `@v7`.

## Что проверяется

| Задача | Что делает | Зачем |
|---|---|---|
| `lint` | `ruff check .` | замечаний сейчас ноль — любое новое видно сразу |
| `unit` | `pytest tests -q` на Python 3.10 / 3.11 / 3.12 | 39 быстрых проверок, ~10 секунд |
| `smoke` | `python tests/smoke_test.py` | 617 сквозных проверок протокола, CLI и вкладок |
| `windows` | проверка `.bat` (CRLF + только ASCII) на `windows-latest` + быстрые тесты | именно этот дефект ломал проект сразу после клонирования |
| `package` | `python -m build`, установка колеса, живой ответ `MSP_FC_VARIANT = BTFL` | гарантия, что `pip install` даёт рабочую команду `bfemu` |

Все задачи кроме `windows` выполняются на Linux; `windows` нужен ровно потому, что
дефект с концами строк воспроизводится только там.

Локально то же самое:

```bash
pip install -e ".[dev]"
ruff check .
pytest tests -q
python tests/smoke_test.py
```
