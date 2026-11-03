"""Самодостаточный HTML-отчёт парного опыта SITL."""
import html
import json

COLORS = ('#66dec6', '#ffb85c', '#6ab5ff', '#f487ad')


def esc(value):
    return html.escape(str(value), quote=True)


def chart(title, unit, curves, *, markers=()):
    """Все PID-отсчёты отображаются без прореживания; отдельная шкала на график."""
    samples = [value for _, _, points in curves for _, value in points]
    low, high = min(samples), max(samples)
    pad = (high-low)*.06 if high != low else max(abs(high)*.06, 1)
    low, high = low-pad, high+pad
    end = max(points[-1][0] for _, _, points in curves)
    parts = [f'<section><h3>{esc(title)}</h3><svg viewBox="0 0 1000 250" role="img" '
             f'aria-label="{esc(title)}; горизонталь — время, секунды; вертикаль — {esc(unit)}">',
             '<path d="M75 12V210H985" stroke="#98a9b9" fill="none"/>']
    for i in range(5):
        y = 210-i*49.5
        val = low+(high-low)*i/4
        parts.append(f'<path d="M75 {y:.1f}H985" stroke="#293642"/><text x="70" y="{y+4:.1f}" text-anchor="end">{val:.2f}</text>')
    for i in range(5):
        x = 75+i*227.5
        parts.append(f'<text x="{x:.1f}" y="230" text-anchor="middle">{end*i/4:.2f}</text>')
    for _, color, points in curves:
        path = ' '.join(f'{75+910*t/end:.2f},{210-198*(v-low)/(high-low):.2f}' for t, v in points)
        parts.append(f'<polyline points="{path}" fill="none" stroke="{color}" stroke-width="1.3"/>')
    for condition, time_s, color, offset in markers:
        if not 0 <= time_s <= end:
            raise ValueError('Момент максимального отклика за пределами графика.')
        x = 75+910*time_s/end
        parts.append(f'<line class="peak-marker" data-condition="{esc(condition)}" x1="{x:.2f}" x2="{x:.2f}" '
                     f'y1="12" y2="210" stroke="{color}" stroke-width="2" stroke-dasharray="5 5" '
                     f'stroke-dashoffset="{offset}"><title>Пик {esc(condition)}: {time_s:.4f} с</title></line>')
    parts.append(f'<text x="530" y="247" text-anchor="middle">Время от начала команды, с</text></svg><p>По вертикали: {esc(unit)}.</p><div class="legend">')
    parts.extend(f'<span style="color:{color}">■ {esc(label)}</span>' for label, color, _ in curves)
    parts.extend(f'<span style="color:{color}"><i aria-hidden="true" style="display:inline-block;width:1.4em;'
                 f'border-top:2px dashed {color};vertical-align:middle"></i> Пик {esc(condition)} · {time_s:.4f} с</span>'
                 for condition, time_s, color, _ in markers)
    parts.append('</div></section>')
    return ''.join(parts)


def technical_content(report):
    """Технические сведения текущего опыта для веб-страницы и скачиваемого отчёта."""
    header = report['header']
    return (
        '<h3>Парный эксперимент Betaflight SITL</h3>'
        '<p>Задание, гироскоп и P/I/D/F записываются в каждом цикле расчёта PID. '
        'Это не MSP-опрос и не данные Blackbox. На каждый кадр приходится 32 отсчёта PID; '
        'снимки моторов доступны только в конце кадра.</p>'
        '<p>Показатели и графики относятся к первому успешному повтору каждой настройки. '
        'Допуски заданы заранее и не являются критериями пригодности для полёта. '
        'В снимке максимального отклика виден один PID-цикл; на движение также влияют '
        'другие составляющие, микшер и динамика модели.</p>'
        '<p>Остальные PID-настройки и фильтры — заводские для этой сборки. До ARM аппарат '
        'удерживается на стенде; после ARM движется без модели земли. Воздух, батарея и '
        'шум датчиков не моделируются. Проверяются физическое состояние, приводы и '
        'снимок PID перед командой, но не все скрытые состояния FC.</p>'
        '<p>Betaflight 4.5.4, ревизия <code>'+esc(header['revision'])+'</code>. '
        'SHA-256 исследовательской сборки SITL: <code>'+esc(header['simclock_elf_sha256'])+'</code>. '
        'Планировщик и часы изменены, PID инструментирован; инициализация и MSP-настройка '
        'происходят по времени хоста.</p>'
        '<details><summary>Полная запись условий, модели и допусков</summary><pre>'+
        esc(json.dumps({'header': header, 'tolerances': report['tolerances'], 'failures': report['failures']},
                       ensure_ascii=False, indent=2))+'</pre></details>'
    )


def render(report, *, include_technical=True):
    h = report['header']
    title = 'Парное сравнение Betaflight SITL в заданной модели'
    before = h['pid']
    after = h['compare_pid']
    def pid_label(value):
        return f"P {value['p']}, I {value['i']}, D Max {value['d']}"
    status = ('Повторяемость в заданном допуске' if report['status'] == 'reproducible'
              else 'Результат не воспроизводится — сравнение недоступно')
    out = [f'''<!doctype html><html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title>
<style>body{{font:16px/1.5 system-ui,sans-serif;background:#10171f;color:#e9f2f4;margin:auto;max-width:1100px;padding:24px}}h1{{font-size:26px}}h2{{margin-top:2em}}section,article{{background:#192530;border:1px solid #344757;border-radius:10px;padding:16px;margin:16px 0}}p{{color:#cad5db}}svg{{width:100%;height:auto;background:#0e171f}}svg text{{fill:#cad5db;font:12px system-ui}}.legend{{display:flex;gap:20px;flex-wrap:wrap}}table{{border-collapse:collapse;width:100%}}th,td{{text-align:left;padding:9px;border-bottom:1px solid #344757}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}.ok{{color:#80dfbe}}.warning{{color:#ffbf8a}}</style></head><body>
<h1>{title}</h1><p>Результат рассчитан в выбранной модели с кодом регулятора Betaflight.</p>
<article><h2 class="{'ok' if report['status'] == 'reproducible' else 'warning'}">{status}</h2><p>{esc(report['reason'])}</p>
<p>Ось: {esc(h['axis'].upper())}. До: {esc(pid_label(before))}. После: {esc(pid_label(after))}. Повторов каждой настройки: {esc(h['repeats'])}.</p>
</article>''']
    if report.get('spread'):
        out.append('<h2>Разброс повторов</h2><p>Каждая колонка показывает различия между повторами одной настройки, а не между настройками. Указаны максимальные различия по всем осям; RMS рассчитан для гироскопа выбранной оси.</p><table><tr><th>Показатель</th><th>До</th><th>После</th><th>Допуск</th></tr>')
        for key, label in (('gyro_dps', 'Гироскоп, °/с'), ('setpoint_dps', 'Задание, °/с'),
                           ('pid', 'P/I/D/F/Sum, внутренние единицы FC'), ('motor', 'Команда мотора, б/р'),
                           ('position_m', 'Положение, м'), ('velocity_m_s', 'Скорость, м/с'),
                           ('orientation', 'Компонента кватерниона, б/р'),
                           ('angular_velocity_rad_s', 'Угловая скорость тела, рад/с'),
                           ('rms_selected_gyro_dps', 'RMS гироскопа выбранной оси, °/с')):
            left, right = (report['spread'][side][key] for side in ('before', 'after'))
            tolerance = report['tolerances'].get(key)
            limit = f'{tolerance:g}' if tolerance is not None else 'Для сведения'
            out.append(f'<tr><td>{esc(label)}</td><td>{left:.6g}</td><td>{right:.6g}</td><td>{limit}</td></tr>')
        out.append('</table>')
    if report['status'] == 'reproducible':
        out.append('<h2>Измерения на интервале команды</h2><p>Цель — среднее задания за последние 100 мс команды. Перерегулирование относительно этой цели; установление — первое время, после которого ошибка относительно цели остаётся в ±5% до конца команды. «Не установилось» означает, что условие не выполнено в этом окне. Интегральная ошибка — интеграл модуля разности текущего задания и гироскопа. Насыщение — число конечных снимков кадров с командой любого мотора 0 или 1; промежуточные команды моторов между снимками не наблюдаются.</p>')
        labels = (('target_dps', 'Цель, °/с'), ('overshoot_percent', 'Перерегулирование, %'),
                  ('settling_s', 'Время установления, с'), ('integral_abs_error_deg', 'Интегральная абсолютная ошибка, °'),
                  ('saturated_frames', 'Кадры насыщения моторов'), ('saturated_time_s', 'Время насыщения, с'))
        out.append('<table><tr><th>Показатель</th><th>До</th><th>После</th></tr>')
        for key, label in labels:
            def display(value, key=key):
                return ('Не установилось' if key == 'settling_s' else 'Не определено') if value is None else (f'{value:.4f}' if isinstance(value, float) else str(value))
            out.append(f'<tr><td>{esc(label)}</td><td>{display(report["metrics"]["before"][key])}</td><td>{display(report["metrics"]["after"][key])}</td></tr>')
        out.append('</table><h3>Что видно в момент максимального отклика</h3>')
        for name, label in (('before', 'До'), ('after', 'После')):
            peak = report['metrics'][name]['peak_sample']
            if peak is None:
                out.append(f'<p>{label}: задание близко к нулю, поэтому пик относительно направления задания не определён.</p>')
                continue
            t, setpoint, gyro, p, i, d, f, total = peak
            error = setpoint-gyro
            text = (f'{label}: через {t:.4f} с задание {setpoint:.2f} °/с, гироскоп {gyro:.2f} °/с, '
                    f'разница {error:.2f} °/с. Внутренние значения FC: P {p:.2f}, I {i:.2f}, '
                    f'D {d:.2f}, F {f:.2f}, Sum {total:.2f}. ')
            if abs(p) > 1e-6 and abs(error) > 1e-6 and p*error > 0:
                text += 'P направлена по знаку ошибки и в этот момент стремится уменьшить расхождение. '
            if abs(p) > 1e-6 and abs(i) > 1e-6 and p*i < 0:
                text += 'I имеет противоположный P знак и частично компенсирует её вклад в сумму. '
            out.append('<p>'+esc(text)+'</p>')
        before, after = report['curves']['before'], report['curves']['after']
        # Противоположные фазы пунктира позволяют увидеть обе отметки, даже когда их время совпадает.
        markers = [(label, report['metrics'][name]['peak_sample'][0], color, offset)
                   for name, label, color, offset in (('before', 'до', '#c39aff', 0),
                                                      ('after', 'после', '#ffffff', 5))
                   if report['metrics'][name]['peak_sample'] is not None]
        out.append('<p>Показатели и графики показывают первый повтор каждой настройки; все повторы проверены на совпадение.</p><h2>Телеметрия регулятора</h2><p>Задание и гироскоп показаны в градусах в секунду, составляющие PID — во внутренних единицах FC.' +
                   (' Пунктир отмечает моменты максимального отклика, описанные выше.' if markers else '') + '</p>')
        out.append(chart('Задание и гироскоп выбранной оси', '°/с', [
            ('Задание до', COLORS[0], [(r[0], r[1]) for r in before]),
            ('Гироскоп до', COLORS[1], [(r[0], r[2]) for r in before]),
            ('Задание после', COLORS[2], [(r[0], r[1]) for r in after]),
            ('Гироскоп после', COLORS[3], [(r[0], r[2]) for r in after])], markers=markers))
        for index, name in enumerate(('P', 'I', 'D', 'F'), 3):
            out.append(chart(f'{name} · составляющая PID', 'внутренние единицы FC', [
                (f'{name} до', COLORS[0], [(r[0], r[index]) for r in before]),
                (f'{name} после', COLORS[1], [(r[0], r[index]) for r in after])], markers=markers))
    if include_technical:
        out.append('<details id="pair-technical"><summary>Технические сведения</summary>' +
                   technical_content(report) + '</details>')
    out.append('</body></html>')
    return ''.join(out)
