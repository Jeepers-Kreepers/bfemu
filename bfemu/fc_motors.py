"""Двигатели, ESC и сервоприводы. Методы VirtualFC.

Примесь использует общее состояние контроллера; создание объекта — через bfemu.fc.VirtualFC.
"""
from __future__ import annotations

import math
import time
from .fc_constants import (
    CHANNEL_FORWARDING_DISABLED,
    DEFAULT_SERVO_MAX,
    DEFAULT_SERVO_MIDDLE,
    DEFAULT_SERVO_MIN,
    INPUT_SOURCE_COUNT,
    MAX_SERVO_RULES,
    MAX_SUPPORTED_MOTORS,
    MAX_SUPPORTED_SERVOS,
)


class MotorsMixin:

    # ------------------------------------------------------------------ модель моторов
    # Вкладка «Моторы» — единственное место, где эмулятор должен быть «физическим»: слайдеры
    # задают газ, а MSP_MOTOR (104) и MSP_MOTOR_TELEMETRY (139) обязаны на это живо откликаться.
    # Обороты/температура/ток/напряжение считаются по простой модели мотора и ESC:
    #   • обороты догоняют задание с постоянной времени раскрутки (и медленнее — при остановке);
    #   • температура греется от газа (≈1.15 °C/с на полном) и остывает к температуре воздуха;
    #   • ток ESC = 0.35 А + gas² · 12 А, напряжение — напряжение батареи минус просадка от
    #     суммарного тока (0.010 Ом), расход интегрируется в мА·ч.
    MOTOR_MAX_RPM = 24000.0          # 2207 ~2500 KV на 4S, пары полюсов 7 → eRPM ≈ 168 тыс.
    MOTOR_SPOOL_TAU_UP = 0.10        # с — раскрутка оборотов
    MOTOR_SPOOL_TAU_DOWN = 0.25      # с — выбег (винт тормозит медленнее, чем ESC разгоняет)
    MOTOR_AMBIENT_C = 24             # ESC на столе
    MOTOR_HEAT_C_PER_S = 1.8         # нагрев на полном газу, °C/с
    MOTOR_COOL_TAU_S = 25.0          # постоянная остывания → 24 + 45 ≈ 69 °C на полном газу
    MOTOR_CURRENT_IDLE_A = 0.35      # холостой ток ESC
    MOTOR_CURRENT_FULL_A = 12.0      # ток мотора на полном газу
    MOTOR_PACK_RESISTANCE_OHM = 0.010
    MOTOR_INVALID_PCT_NO_TELEMETRY = 10000   # msp.c: 10000 = «100.00 %» — телеметрии нет

    def servo_lines(self, only_diff=False):
        """Строки 'servo i min max middle rate forward' — printServo() из cli.c.

        Формат ровно как в прошивке (4.5.4): ``servo %u %d %d %d %d %d``.
        """
        defaults = self._default_servo_params()
        out = []
        for index, params in enumerate(self.servo_params):
            if only_diff and params == defaults[index]:
                continue
            out.append(f"servo {index} {params['min']} {params['max']} {params['middle']} "
                       f"{params['rate']} {params['forwardFromChannel']}")
        return out

    def smix_lines(self, only_diff=False):
        """Строки 'smix ...' — printServoMix() из cli.c.

        Как и в прошивке, список обрывается на первом правиле с rate == 0:
        заводские правила (нули) поэтому не печатаются вовсе.
        """
        out = []
        for index, rule in enumerate(self.servo_mix_rules):
            if rule["rate"] == 0:
                break
            if only_diff and self._default_servo_mix_rules()[index] == rule:
                continue
            out.append(f"smix {index} {rule['targetChannel']} {rule['inputSource']} {rule['rate']} "
                       f"{rule['speed']} {rule['min']} {rule['max']} {rule['box']}")
        return out

    def smix_reverse_lines(self, only_diff=False):
        """Строки 'smix reverse <servo> <source> r' — направления сервоприводов."""
        out = []
        for index, params in enumerate(self.servo_params):
            for source in range(INPUT_SOURCE_COUNT):
                if params["reversedSources"] & (1 << source):
                    out.append(f"smix reverse {index} {source} r")
        return out

    @staticmethod
    def _default_custom_motor_mix() -> list:
        """customMotorMixer[] пуст: printMotorMix() останавливается на нулевом газе — строк нет."""
        return []

    @staticmethod
    def _default_servo_params() -> list:
        """Заводские параметры сервоприводов (pgResetFn_servoParams в flight/servos.c)."""
        return [
            {
                "min": DEFAULT_SERVO_MIN,
                "max": DEFAULT_SERVO_MAX,
                "middle": DEFAULT_SERVO_MIDDLE,
                "rate": 100,
                "forwardFromChannel": CHANNEL_FORWARDING_DISABLED,
                "reversedSources": 0,
            }
            for _ in range(MAX_SUPPORTED_SERVOS)
        ]

    @staticmethod
    def _default_servo_mix_rules() -> list:
        """Правила микшера сервоприводов: по умолчанию всё в нулях (customServoMixers)."""
        return [
            {"targetChannel": 0, "inputSource": 0, "rate": 0, "speed": 0, "min": 0, "max": 0, "box": 0}
            for _ in range(MAX_SERVO_RULES)
        ]

    # ------------------------------------------------------------------ моторы: модель и MSP
    def motor_telemetry_active(self) -> bool:
        """Есть ли данные ESC: бидирекшн DShot или датчик ESC — то же условие, что в msp.c."""
        return bool(self._setting("dshot_bidir", 0)) or "ESC_SENSOR" in self.features

    def motor_invalid_percent(self, index: int) -> int:
        """Доля «битых» кадров ESC в сотых долях процента (как её отдаёт MSP_MOTOR_TELEMETRY).

        Модель та же, что и у вкладки «Моторы»: у вращающегося мотора ошибок чуть больше.
        """
        rpm = int(round(self.motor_rpm[index]))
        invalid = 3 + (5 if rpm > 500 else 0) + int(2 * abs(math.sin(time.time() * 3.1 + index)))
        return min(invalid, 99)

    def _motor_count(self) -> int:
        """Число моторов борта (getMotorCount()): из настройки motor_count, 4 по умолчанию."""
        return max(1, min(MAX_SUPPORTED_MOTORS, self._setting("motor_count", 4)))

    def _motors_step(self) -> None:
        """Один шаг «физики» моторов: обороты, температура, ток, напряжение, расход.

        Вызывается перед каждым ответом MSP_MOTOR/MSP_MOTOR_TELEMETRY — вкладка опрашивает
        их каждые 50 мс, поэтому шаг получается ровно такой, как нужно для живых цифр.
        """
        now = time.time()
        dt = max(0.0, min(2.0, now - self._motors_t))   # до 2 с «простоя» учитываем целиком
        self._motors_t = now
        if dt <= 0.0:
            return
        if self.fs_demo:
            # «Демо failsafe»: часы демо (с масштабом показа) и машина фаз прошивки —
            # страж-время, «Этап 1»/«Этап 2», посадка или спасение. Выходы моторов берём
            # из сценария failsafe и пишем в motor_disarmed — как mixer.c на взведённом борде.
            self._fs_step(dt)
            self.motor_disarmed = self._fs_outputs(now)
        elif self.sim_flight:
            # «Виртуальный полёт»: сначала шаг сценария (фазы, посадка, замена батареи),
            # затем выходы моторов — пишем их в motor_disarmed, ровно как mixer.c на
            # взведённом борде. MSP_MOTOR (104) и вкладка «Моторы» показывают те же числа.
            self._sim_flight_step(now)
            self.motor_disarmed = self._sim_flight_outputs(now)
        # Интегрируем мелкими шагами по 5 мс: если между опросами прошло много времени
        # (вкладка была закрыта), мотор успевает раскрутиться физически правильно, но
        # обороты не «прыгают» через промежуточные значения.
        steps = max(1, min(400, int(dt / 0.005) + 1))
        for _ in range(steps):
            self._motors_integrate(dt / steps)

    def _motors_integrate(self, dt: float) -> None:
        """Шаг модели моторов на dt секунд (вызывается из _motors_step)."""
        # Напряжение «покоя» — без просадки: её считаем здесь, в одном месте
        # (иначе просадка вычиталась бы дважды — ещё и в _battery_voltage()).
        # «Покой» — с учётом разряда: у разряженной батареи он ниже (см. _pack_rest_voltage).
        rest_v = self._pack_rest_voltage()
        total_current = 0.0
        for i in range(MAX_SUPPORTED_MOTORS):
            output = max(1000.0, min(2000.0, self.motor_disarmed[i]))
            self.motor_output[i] = output
            gas = max(0.0, min(1.0, (output - 1000.0) / 1000.0))

            target_rpm = gas * self.MOTOR_MAX_RPM
            tau = self.MOTOR_SPOOL_TAU_UP if target_rpm >= self.motor_rpm[i] else self.MOTOR_SPOOL_TAU_DOWN
            self.motor_rpm[i] += (target_rpm - self.motor_rpm[i]) * min(1.0, dt / tau)
            if self.motor_rpm[i] < 20.0:
                self.motor_rpm[i] = 0.0

            # Ток и нагрев считаем по ФАКТИЧЕСКИМ оборотам, а не по заданию: раскрученный
            # винт продолжает грузить ESC и на выбеге — ток спадает вместе с оборотами.
            gas_eff = min(1.0, self.motor_rpm[i] / self.MOTOR_MAX_RPM)
            heat = self.MOTOR_HEAT_C_PER_S * gas_eff * gas_eff
            cool = (self.motor_temp_c[i] - self.MOTOR_AMBIENT_C) / self.MOTOR_COOL_TAU_S
            self.motor_temp_c[i] = max(self.MOTOR_AMBIENT_C, self.motor_temp_c[i] + (heat - cool) * dt)

            current = (self.MOTOR_CURRENT_IDLE_A + self.MOTOR_CURRENT_FULL_A * gas_eff * gas_eff) if gas_eff > 0.005 else 0.0
            self.motor_current_a[i] = current
            total_current += current
            self.motor_mah[i] += current * self._sim_scaled_dt(dt) / 3.6   # А·с → мА·ч

        # Напряжение на ESC — напряжение батареи минус просадка от общего тока
        self.motor_voltage_v = max(0.0, rest_v - total_current * self.MOTOR_PACK_RESISTANCE_OHM) if rest_v else 0.0

    def motor_test_active(self) -> bool:
        """Идёт ли тест моторов: выход хотя бы одного мотора выше «стопа» (1000)."""
        return any(value > 1001.0 for value in self.motor_disarmed)

    def motors_state(self) -> dict:
        """Снимок моторов для монитора эмулятора (/api/state)."""
        self._motors_step()
        count = self._motor_count()
        motors = []
        for i in range(count):
            motors.append({
                "index": i + 1,
                "output": int(round(self.motor_output[i])),
                "rpm": int(round(self.motor_rpm[i])),
                "temp_c": int(round(self.motor_temp_c[i])),
                "current_a": round(self.motor_current_a[i], 2),
                "mah": round(self.motor_mah[i], 1),
            })
        return {
            "count": count,
            "test_active": self.motor_test_active(),
            "sim_flight": self.sim_flight,
            "voltage_v": round(self.motor_voltage_v, 2),
            "dshot_telemetry": bool(self._setting("dshot_bidir", 0)),
            "esc_sensor": "ESC_SENSOR" in self.features,
            "protocol": self._setting("motor_pwm_protocol", 7),
            "motors": motors,
        }

    def _h_motor(self, w, r):
        """MSP_MOTOR (104): msp.c всегда пишет ВОСЕМЬ u16 — для выключенных моторов нули,
        для остальных текущий выход `motor[]` (когда борт не взведён — motor_disarmed[])."""
        self._motors_step()
        count = self._motor_count()
        for i in range(MAX_SUPPORTED_MOTORS):
            w.u16(int(round(self.motor_output[i])) if i < count else 0)

    def _h_servo(self, w, r):
        """MSP_SERVO (103): 8 текущих значений сервоприводов.

        Как servoMixer() в прошивке: если у сервопривода задано перенаправление
        канала, отдаём значение этого RC-канала, иначе — «середину».
        """
        channels = self._rc_channel_values()
        for index in range(MAX_SUPPORTED_SERVOS):
            params = self.servo_params[index]
            forward = params["forwardFromChannel"]
            if forward != CHANNEL_FORWARDING_DISABLED and forward < len(channels):
                w.u16(channels[forward])
            else:
                w.u16(params["middle"])

    def _rc_channel_values(self) -> list:
        """Значения RC-каналов (на столе: стики по центру, газ внизу, AUX выключены)."""
        # Порядок A/E/R/T, как в MSP_RC: значения берём у виртуальной аппаратуры, чтобы
        # тумблеры на вкладке «Приёмник» и таблица «Серво» показывали одно и то же.
        return list(self.rc_channels)

    def _h_servo_configurations(self, w, r):
        """MSP_SERVO_CONFIGURATIONS (120): 8 × 12 байт — min, max, middle, rate, forward, reversed.

        Вкладка «Сервоприводы» считает набор непустым только если длина кратна 12.
        """
        for params in self.servo_params:
            w.u16(params["min"])
            w.u16(params["max"])
            w.u16(params["middle"])
            w.u8(params["rate"])
            w.u8(params["forwardFromChannel"])
            w.u32(params["reversedSources"])

    def _h_set_servo_configuration(self, w, r):
        """MSP_SET_SERVO_CONFIGURATION (212): index + те же 12 байт (msp.c требует ровно 13)."""
        data = bytes(r.rest())
        if len(data) < 13:
            return b""
        index = data[0]
        if index >= MAX_SUPPORTED_SERVOS:
            self._log(f"[MSP] SERVO: неверный индекс {index}, запись отклонена")
            return b""
        self.servo_params[index] = {
            "min": int.from_bytes(data[1:3], "little"),
            "max": int.from_bytes(data[3:5], "little"),
            "middle": int.from_bytes(data[5:7], "little"),
            "rate": data[7],
            "forwardFromChannel": data[8],
            "reversedSources": int.from_bytes(data[9:13], "little"),
        }
        params = self.servo_params[index]
        forward = params["forwardFromChannel"]
        self._log(
            f"[MSP] SERVO {index + 1}: {params['min']}/{params['middle']}/{params['max']}, "
            f"rate {params['rate']}, перенаправление "
            f"{'выключено' if forward == CHANNEL_FORWARDING_DISABLED else forward + 1}"
        )
        return b""   # пустой ACK: вкладка «Сервоприводы» ждёт ответа на каждую запись

    def _h_servo_mix_rules(self, w, r):
        """MSP_SERVO_MIX_RULES (241): 16 правил по 7 байт."""
        for rule in self.servo_mix_rules:
            w.u8(rule["targetChannel"]).u8(rule["inputSource"]).u8(rule["rate"]).u8(rule["speed"])
            w.u8(rule["min"]).u8(rule["max"]).u8(rule["box"])

    def _h_set_servo_mix_rule(self, w, r):
        """MSP_SET_SERVO_MIX_RULE (242): index + правило из 7 байт."""
        data = bytes(r.rest())
        if len(data) < 8:
            return b""
        index = data[0]
        if index >= MAX_SERVO_RULES:
            return b""
        self.servo_mix_rules[index] = {
            "targetChannel": data[1], "inputSource": data[2], "rate": data[3], "speed": data[4],
            "min": data[5], "max": data[6], "box": data[7],
        }
        return b""

    def _h_motor_telemetry(self, w, r):
        """MSP_MOTOR_TELEMETRY (139): число моторов, затем по 13 байт на мотор —
        обороты (u32), доля ошибок (u16, 10000 = 100.00 %), температура (u8, °C),
        напряжение (u16, 0.01 В), ток (u16, 0.01 А), расход (u16, мА·ч).

        Раскладка ровно как в msp.c 4.5.4 и как её читает MSPHelper.js. Данные живые:
        слайдер на вкладке «Моторы» → растут обороты, ток и температура, а напряжение
        проседает тем сильнее, чем больше суммарный ток четырёх ESC.
        """
        self._motors_step()
        count = self._motor_count()
        w.u8(count)
        for i in range(count):
            rpm = int(round(self.motor_rpm[i]))
            if self.motor_telemetry_active():
                invalid = self.motor_invalid_percent(i)     # 0.03…0.10 %
            else:
                invalid = self.MOTOR_INVALID_PCT_NO_TELEMETRY
            w.u32(rpm)
            w.u16(invalid)
            w.u8(int(round(self.motor_temp_c[i])))
            w.u16(int(round(self.motor_voltage_v * 100)))
            w.u16(int(round(self.motor_current_a[i] * 100)))
            w.u16(int(self.motor_mah[i]) & 0xFFFF)

    def _h_motor_config(self, w, r):
        """MSP_MOTOR_CONFIG (131): 10 байт — как pgResetFn_motorConfig + getMotorCount().

        Значения берутся из CLI-настроек min_throttle/max_throttle/min_command, поэтому
        вкладка «PID» («Параметры газа и моторов»), вкладка «Моторы» и CLI не расходятся.
        """
        s = lambda n, d=0: int(self.settings.get(n, d))
        w.u16(s("min_throttle", 1070)).u16(s("max_throttle", 2000)).u16(s("min_command", 1000))
        w.u8(int(s("motor_count", 4)))
        w.u8(s("motor_poles", 14))
        # useDshotTelemetry = DEFAULT_DSHOT_TELEMETRY: у 4.5.4 по умолчанию выключено
        # (настройка dshot_bidir), поэтому «Динамический Idle» во вкладке выключен — как на железе.
        w.u8(1 if s("dshot_bidir", 0) else 0)
        w.u8(1 if "ESC_SENSOR" in self.features else 0)   # esc_sensor = featureIsEnabled(FEATURE_ESC_SENSOR)

    def _h_set_motor_config(self, w, r):
        """MSP_SET_MOTOR_CONFIG (222): 10 байт — раскладка как у ответа 131."""
        d = bytes(r.rest())
        if len(d) < 10:
            return b""
        u16 = lambda o: int.from_bytes(d[o:o + 2], "little")
        self.settings["min_throttle"], self.settings["max_throttle"] = u16(0), u16(2)
        self.settings["min_command"] = u16(4)
        self.settings["motor_poles"] = d[7]
        self.settings["dshot_bidir"] = 1 if d[8] else 0
        self._log(f"[MSP] MOTOR_CONFIG сохранён: min_throttle={self.settings['min_throttle']}, "
                  f"max_throttle={self.settings['max_throttle']}, min_command={self.settings['min_command']}")
        return b""

    def _h_motor_3d_config(self, w, r):
        """MSP_MOTOR_3D_CONFIG (124): 6 байт — deadband3d_low/high и neutral3d
        (msp.c: flight3DConfig(); заводские значения из PG_MOTOR_3D_CONFIG)."""
        w.u16(self._setting("3d_deadband_low", 1406))
        w.u16(self._setting("3d_deadband_high", 1514))
        w.u16(self._setting("3d_neutral", 1460))

    def _h_set_motor_3d_config(self, w, r):
        """MSP_SET_MOTOR_3D_CONFIG (217): пишем те же три настройки в «флеш»."""
        data = bytes(r.rest())
        if len(data) >= 6:
            self._store_setting("3d_deadband_low", int.from_bytes(data[0:2], "little"))
            self._store_setting("3d_deadband_high", int.from_bytes(data[2:4], "little"))
            self._store_setting("3d_neutral", int.from_bytes(data[4:6], "little"))
            self._log("[MSP] MOTOR_3D_CONFIG сохранён: deadband "
                      f"{self.settings.get('3d_deadband_low')}/{self.settings.get('3d_deadband_high')}, "
                      f"neutral {self.settings.get('3d_neutral')}")
        return b""

    def _h_motor_output_reordering(self, w, r):
        """MSP2_MOTOR_OUTPUT_REORDERING (0x3001): u8 count + по u8 на мотор.

        msp.c пишет MAX_SUPPORTED_MOTORS (8) и массив motorOutputReordering[],
        заводское значение — 0,1,2,…,7 (pg/motor.c: reordering[i] = i).
        """
        w.u8(MAX_SUPPORTED_MOTORS)
        for i in range(MAX_SUPPORTED_MOTORS):
            w.u8(self.motor_output_reordering[i] & 0xFF)

    def _h_set_motor_output_reordering(self, w, r):
        """MSP2_SET_MOTOR_OUTPUT_REORDERING (0x3002): сохраняем порядок моторов."""
        data = bytes(r.rest())
        if len(data) >= 1 + MAX_SUPPORTED_MOTORS:
            self.motor_output_reordering = list(data[1:1 + MAX_SUPPORTED_MOTORS])
            self._log(f"[MSP] порядок выходов моторов сохранён: {self.motor_output_reordering}")
        return b""

    def _h_send_dshot_command(self, w, r):
        """MSP2_SEND_DSHOT_COMMAND (0x3003): команда ESC. Вкладка «Моторы» шлёт так
        «включить расширенную телеметрию» (команда 13) при входе в тест и «стоп»
        (команда 12) при выходе. Прошивка отвечает пустым ответом — логируем для монитора.
        """
        data = bytes(r.rest())
        if len(data) >= 3:
            command_type, motor, count = data[0], data[1], data[2]
            commands = list(data[3:3 + count])
            self.dshot_commands_log.append({
                "type": command_type, "motor": motor, "commands": commands,
                "time": time.strftime("%H:%M:%S"),
            })
            self.dshot_commands_log = self.dshot_commands_log[-10:]
            self._log(f"[MSP] DSHOT-команды ESC (мотор {motor}): {commands}")
        return b""

    def _h_set_motor(self, w, r):
        """MSP_SET_MOTOR (214) — тест моторов со слайдеров вкладки «Моторы».

        msp.c 4.5.4: `motor_disarmed[i] = motorConvertFromExternal(...)` для getMotorCount()
        моторов, лишние байты игнорируются. Значения НЕ сбрасываются по таймауту: на железной
        плате мотор продолжает крутиться на выставленном газе, пока Configurator не пришлёт
        нули (кнопка «Стоп» / отпущенный ползунок мастера кладёт 1000). Останавливает их
        только MSP_SET_MOTOR с mincommand, выключение теста или перезагрузка платы.
        """
        count = self._motor_count()
        data = bytes(r.rest())
        values = []
        for i in range(count):
            offset = i * 2
            if offset + 2 > len(data):
                break
            values.append(max(1000, min(2000, int.from_bytes(data[offset:offset + 2], "little"))))
        if self.sim_flight:
            # Идёт виртуальный полёт: газ держит сценарий (как mixer.c на взведённом борде),
            # поэтому слайдеры вкладки «Моторы» сейчас ничего не меняют — иначе моторы
            # «дёргались» бы между сценарием и ползунком.
            self.writes_log.append(f"motors: {values} (игнор: виртуальный полёт)")
            return b""
        if values:
            for i, value in enumerate(values):
                self.motor_disarmed[i] = float(value)
        self.writes_log.append(f"motors: {values}")
        return b""
