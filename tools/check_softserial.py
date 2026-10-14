"""Сквозная проверка: включение SOFTSERIAL во вкладке «Конфигурация» и появление
SOFTSERIAL1/2 во вкладке «Порты» — по TCP, как это видит Configurator."""
import json
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, "/home/user/bfemu")
from bfemu.msp import encode_v1, encode_v2

eeprom = Path(tempfile.mkdtemp()) / "eeprom.json"
port, http_port = 16011, 16012
proc = subprocess.Popen(
    [sys.executable, "-m", "bfemu", "--tcp", f"127.0.0.1:{port}", "--http", str(http_port), "--eeprom", str(eeprom)],
    cwd="/home/user/bfemu", stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
)
time.sleep(2.5)


def reply(sock, code, payload=b"", v2=False):
    sock.sendall(encode_v2(code, payload) if v2 else encode_v1(code, payload))
    buf = b""
    deadline = time.time() + 2
    while time.time() < deadline:
        try:
            buf += sock.recv(65536)
        except TimeoutError:
            break
        for i in range(len(buf) - 5):
            if buf[i] == 0x24 and buf[i + 1] == 0x4D and buf[i + 2] == 0x3E:
                size = buf[i + 3]
                if i + 5 + size <= len(buf):
                    return buf[i + 5 : i + 5 + size]
            if buf[i] == 0x24 and buf[i + 1] == 0x58 and buf[i + 2] == 0x3E:
                size = buf[i + 6] | (buf[i + 7] << 8)
                if i + 8 + size <= len(buf):
                    return buf[i + 8 : i + 8 + size]
        time.sleep(0.05)
    return b""


def port_ids(sock):
    payload = reply(sock, 0x1009, v2=True)
    return [payload[1 + 9 * i] for i in range(payload[0])] if payload else []


s = socket.create_connection(("127.0.0.1", port), timeout=3)
s.settimeout(0.4)
api = reply(s, 1)
board = reply(s, 4)
print(f"1) рукопожатие: API {list(api)}, плата {board[:4].decode()}, caps = 0x{board[7]:02x} "
      f"(VCP={bool(board[7] & 1)}, SOFTSERIAL={bool(board[7] & 2)})")
print(f"2) пробуем вкладку «Порты»: MSP_VTX_CONFIG {len(reply(s, 88))} байт")
print(f"   порты при выключенной фиче: {port_ids(s)}")

# биты фич как в прошивке (feature.h) и в Configurator (Features.js):
# SOFTSERIAL = 1 << 6, OSD = 1 << 18, AIRMODE = 1 << 22
bits = {"SOFTSERIAL": 1 << 6, "OSD": 1 << 18, "AIRMODE": 1 << 22}
reply(s, 37, sum(bits.values()).to_bytes(4, "little"))   # галочка SOFTSERIAL
print(f"3) после включения SOFTSERIAL: {port_ids(s)}")
reply(s, 250)                     # кнопка Save
reply(s, 68, bytes([0]))          # Save and Reboot
time.sleep(1.8)
try:
    s.close()
except OSError:
    pass

s2 = socket.create_connection(("127.0.0.1", port), timeout=3)
s2.settimeout(0.4)
reply(s2, 1)
print(f"4) после перезапуска        : {port_ids(s2)}  <- 30 и 31 это SOFTSERIAL1/2")

state = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{http_port}/api/state", timeout=5).read())
print("5) монитор /api/state        :", [(p["name"], p["functions"]) for p in state["ports"]])
print("   CLIP:", state["features"])
s2.close()
proc.terminate()
proc.wait(timeout=10)
