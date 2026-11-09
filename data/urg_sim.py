"""Simulátor lidaru URG-04LX: falešný sériový port odpovídající ve formátu SCIP 2.0.

Formát odpovědí (echo, stav + součet, časová známka, datové řádky po 64 znacích + součet,
prázdný řádek) odpovídá tomu, co program očekává. Obsah odpovědí VV/II a scéna jsou
SMYŠLENÉ (testovací data). Hodnoty PP odpovídají tomu, co URG-04LX podle mé paměti
hlásí (DMAX:5600 atd.) – nejisté; program je stejně čte, nemá je natvrdo.
Kódy odpovědí, které neznám jistě (stav na SCIP2.0, GD bez zapnutého laseru,
neznámý příkaz), jsou označené "jen simulátor".
"""
from __future__ import annotations

import time

import numpy as np

from urg04lx import MAX_DATA_LINE, checksum, encode

PP = {"MODL": "URG-04LX(Hokuyo Automatic Co.,Ltd.)", "DMIN": "20", "DMAX": "5600",
      "ARES": "1024", "AMIN": "44", "AMAX": "725", "AFRT": "384", "SCAN": "600"}
VV = {"VEND": "SIMULATOR", "PROD": "FAKE URG-04LX", "FIRM": "0.0.0", "PROT": "SCIP 2.0",
      "SERI": "SIM0001"}
II = {"MODL": "URG-04LX (simulator)", "LASR": "OFF", "SCSP": "600", "MESM": "simulated",
      "SBPS": "USB", "TIME": "000000", "STAT": "OK"}


def default_scene() -> np.ndarray:
    """Smyšlená scéna pro kroky 0–768: stěna 1000 mm, kostka 500 mm vpředu, chybové kódy."""
    d = np.full(769, 1000, dtype=np.int64)
    d[382:387] = 500        # „kostka 2x2“ přímo vpřed (krok 384 = 0°)
    d[44:48] = 0            # chybový kód 0
    d[700] = 16             # chybový kód 16
    d[600] = 6000           # nad DMAX (5600)
    return d


def info_reply(cmd: str, values: dict) -> str:
    lines = [cmd, "00" + checksum("00")]
    lines += [f"{k}:{v};{checksum(f'{k}:{v}')}" for k, v in values.items()]
    return "\n".join(lines) + "\n\n"


def scan_reply(echo: str, status: str, ts_ms: int, dist: np.ndarray, start: int, end: int,
               cluster: int, n_chars: int, corrupt_line: int | None = None) -> str:
    vals = [int(dist[s:min(s + cluster, end + 1)].min()) for s in range(start, end + 1, cluster)]
    data = "".join(encode(min(v, (1 << (6 * n_chars)) - 1), n_chars) for v in vals)
    rows = [data[i:i + MAX_DATA_LINE] for i in range(0, len(data), MAX_DATA_LINE)]
    rows = [r + checksum(r) for r in rows]
    if corrupt_line is not None and corrupt_line < len(rows):   # změna znaku, součet zůstane
        r = rows[corrupt_line]
        rows[corrupt_line] = ("1" if r[0] != "1" else "2") + r[1:]
    ts = encode(ts_ms % (1 << 24), 4)
    return "\n".join([echo, status + checksum(status), ts + checksum(ts), *rows]) + "\n\n"


class FakeURG:
    """Rozhraní jako serial.Serial: write, read, in_waiting, reset_input_buffer, close."""

    def __init__(self, scene: np.ndarray | None = None, scan_period_s: float = 0.1,
                 corrupt: dict[int, int] | None = None, interrupt_after_scans: int | None = None,
                 silent: bool = False, timeout: float = 0.2):
        self.scene = default_scene() if scene is None else scene
        self.scan_period_s = scan_period_s
        self.corrupt = corrupt or {}           # {pořadí skenu: index datového řádku k poškození}
        self.interrupt_after_scans = interrupt_after_scans
        self.silent = silent                   # neodpovídá vůbec (test timeoutu)
        self.timeout = timeout
        self.t0 = time.monotonic()
        self.out = bytearray()
        self.inbuf = bytearray()
        self.laser = False
        self.md = None                         # běžící MD/MS
        self.scans_sent = 0
        self.history: list[str] = []
        self.is_open = True

    # ---- rozhraní sériového portu
    @property
    def in_waiting(self) -> int:
        self._md_tick(wait=False)
        return len(self.out)

    def write(self, data: bytes) -> int:
        self.inbuf += data
        while b"\n" in self.inbuf:
            line, _, rest = bytes(self.inbuf).partition(b"\n")
            self.inbuf = bytearray(rest)
            self._handle(line.decode("ascii"))
        return len(data)

    def read(self, n: int = 1) -> bytes:
        if not self.out:
            self._md_tick(wait=True)
        if not self.out and not self.md:
            time.sleep(min(self.timeout, 0.01))
        chunk = bytes(self.out[:n])
        del self.out[:n]
        return chunk

    def reset_input_buffer(self) -> None:
        self.out.clear()

    def close(self) -> None:
        self.is_open = False

    # ---- logika lidaru
    def _ms(self) -> int:
        return int((time.monotonic() - self.t0) * 1000)

    def _next_scan(self, echo, status, start, end, cluster, n_chars) -> str:
        if self.interrupt_after_scans is not None and self.scans_sent >= self.interrupt_after_scans:
            self.interrupt_after_scans = None
            raise KeyboardInterrupt
        rep = scan_reply(echo, status, self._ms(), self.scene, start, end, cluster, n_chars,
                         self.corrupt.get(self.scans_sent))
        self.scans_sent += 1
        return rep

    def _handle(self, cmd: str) -> None:
        self.history.append(cmd)
        if self.silent:
            return
        k = cmd[:2]
        if cmd == "QT":
            self.md = None
            self.laser = False
            rep = "QT\n00P\n\n"
        elif cmd == "SCIP2.0":
            rep = "SCIP2.0\n0Ee\n\n"           # stejně jako skutečný lidar (už v SCIP 2.0)
        elif cmd in ("VV", "PP", "II"):
            rep = info_reply(cmd, {"VV": VV, "PP": PP, "II": II}[cmd])
        elif cmd == "BM":
            st = "02" if self.laser else "00"
            self.laser = True
            rep = f"BM\n{st}{checksum(st)}\n\n"
        elif k in ("GD", "GS") and len(cmd) == 12:
            if not self.laser:
                rep = f"{cmd}\n10{checksum('10')}\n\n"   # jen simulátor
            else:
                rep = self._next_scan(cmd, "00", int(cmd[2:6]), int(cmd[6:10]), int(cmd[10:12]) or 1,
                                      3 if k == "GD" else 2)
        elif k in ("MD", "MS") and len(cmd) == 15:
            self.laser = True
            self.md = {"cmd": cmd, "start": int(cmd[2:6]), "end": int(cmd[6:10]),
                       "cluster": int(cmd[10:12]) or 1, "left": int(cmd[13:15]),
                       "n_chars": 3 if k == "MD" else 2, "next": time.monotonic()}
            rep = f"{cmd}\n00P\n\n"
        else:
            rep = f"{cmd}\n0E{checksum('0E')}\n\n"       # jen simulátor
        self.out += rep.encode("ascii")

    def _md_tick(self, wait: bool) -> None:
        m = self.md
        if not m:
            return
        now = time.monotonic()
        if now < m["next"]:
            if not wait:
                return
            time.sleep(min(m["next"] - now, self.timeout))
            if time.monotonic() < m["next"]:
                return
        infinite = m["left"] == 0
        left = 0 if infinite else m["left"] - 1
        echo = m["cmd"][:13] + f"{left:02d}"
        self.out += self._next_scan(echo, "99", m["start"], m["end"], m["cluster"],
                                    m["n_chars"]).encode("ascii")
        m["next"] += self.scan_period_s
        if not infinite:
            m["left"] = left
            if left == 0:
                self.md = None
