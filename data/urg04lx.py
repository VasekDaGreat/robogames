"""Hokuyo URG-04LX(-UG01), protokol SCIP 2.0: měření a převod na mm / stupně.

Použití
  python urg04lx.py --port COM3 --mode single --out mereni1
  python urg04lx.py --port /dev/ttyACM0 --mode n --count 10 --out mereni2
  python urg04lx.py --port COM3 --mode time --seconds 5 --encoding 2 --out mereni3
  python urg04lx.py --sim --mode time --seconds 1 --out test_sim     # bez lidaru (simulátor)
  python urg04lx.py --parse mereni1/raw.txt --out mereni1_znovu      # offline znovu dekódovat

Výstupy v --out: scans.csv, scans.npz, metadata.json, log.txt, raw.txt (vše přijaté z lidaru,
vstup pro --parse), volitelně plot.png (--plot, potřebuje matplotlib).

Čísla kapitol specifikace SCIP 2.0: u žádného pravidla je neznám jistě -> všude "kap. nejisté".
Závislosti: numpy, pyserial (jen pro skutečný port), volitelně matplotlib.
"""
from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import platform
import re
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

log = logging.getLogger("urg04lx")

# =============================================================================
# Konstanty – snadno změnitelné. Vše, co je "nejisté", je tady a ne uvnitř kódu.
# =============================================================================
BAUD = 115200            # u USB (virtuální COM) se rychlost ve skutečnosti nepoužívá
READ_TIMEOUT_S = 1.0     # timeout jednoho čtení z portu
REPLY_TIMEOUT_S = 3.0    # max. čekání na celou odpověď (do prázdného řádku)
MAX_DATA_LINE = 64       # max. počet datových znaků na řádku (bez součtu), kap. nejisté
TIMESTAMP_BITS = 24      # 4 znaky × 6 bitů; čas přetéká po 2^24 ms ≈ 4,66 h, kap. nejisté

CHARS_PER_VALUE = {"GD": 3, "MD": 3, "GS": 2, "MS": 2}   # kap. nejisté
STATUS_DATA = {"GD": {"00"}, "GS": {"00"}, "MD": {"99"}, "MS": {"99"}}  # odpověď s daty
STATUS_ACK = {"MD": {"00"}, "MS": {"00"}}   # potvrzení MD/MS bez dat
BM_OK = {"00", "02"}     # 00 ověřeno na lidaru; 02 = laser už byl zapnutý – nejisté
# Kontrolní součet řádků VV/PP/II: počítá se ze znaků PŘED středníkem.
# Ověřeno na skutečném URG-04LX (FW 3.4.03, 2026-11-09): sedí všechny řádky.
# Pokud by seděla jen varianta se středníkem, řádek přijmeme a zalogujeme to.
INFO_SUM_EXCLUDES_SEMICOLON = True
# Odpověď na SCIP2.0: "0E" = senzor už je v SCIP 2.0 (ověřeno na lidaru); jen se loguje.
CLUSTER_ZERO_AS_ONE = True   # cluster "00" bereme jako 1 – nejisté

# Význam chybových kódů (hodnoty < 20) u URG-04LX.
# Zdroj: URG_SCIP11.pdf, výkres C-42-3320-A, str. 7/8 (dokument pro SCIP 1.1).
# Že platí beze změny i ve SCIP 2.0, je NEJISTÉ (spec. SCIP 2.0 nemáme).
# Používá se JEN pro text v logu; o platnosti bodu rozhoduje DMIN/DMAX z PP.
ERROR_CODES = {
    0: "objekt možná ve vzdálenosti 22 m",
    1: "slabá intenzita odraženého světla", 2: "slabá intenzita odraženého světla",
    3: "slabá intenzita odraženého světla", 4: "slabá intenzita odraženého světla",
    5: "slabá intenzita odraženého světla",
    6: "objekt možná ve vzdálenosti 5,7 m",
    7: "chyba v předchozím i následujícím kroku",
    8: "jiné", 9: "stejný krok měl chybu ve dvou posledních skenech",
    10: "jiné", 11: "jiné", 12: "jiné", 13: "jiné", 14: "jiné", 15: "jiné",
    16: "objekt možná v rozsahu 4096 mm – 5,6 m",
    17: "jiné", 18: "nespecifikováno", 19: "vzdálenost nelze změřit",
}

PP_KEYS = ("DMIN", "DMAX", "ARES", "AMIN", "AMAX", "AFRT", "SCAN")

ECHO_GD = re.compile(r"^(GD|GS)(\d{4})(\d{4})(\d{2})(;.*)?$")
ECHO_MD = re.compile(r"^(MD|MS)(\d{4})(\d{4})(\d{2})(\d)(\d{2})(;.*)?$")

CSV_COLUMNS = ["scan_id", "timestamp_ms", "step", "angle_deg", "distance_mm", "valid",
               "x_mm", "y_mm",
               # sloupce navíc:
               "step_last",    # poslední krok skupiny (cluster > 1)
               "raw_value",    # surová hodnota (i chybový kód); -1 = nelze určit (vadný sken)
               "scan_valid"]   # 0 = celý sken nevalidní (součet, počet hodnot, stav)


# =============================================================================
# Kódování SCIP 2.0
# =============================================================================
def checksum(s: str) -> str:
    """Součet ASCII hodnot, spodních 6 bitů, + 0x30. Kap. nejisté."""
    return chr((sum(s.encode("ascii", errors="replace")) & 0x3F) + 0x30)


def decode(chars: str) -> int:
    """Každý znak − 0x30 = 6 bitů, skládá se od nejvyššího. Kap. nejisté."""
    v = 0
    for c in chars:
        v = (v << 6) | (ord(c) - 0x30)
    return v


def encode(value: int, n: int) -> str:
    """Opak decode (pro simulátor a testy)."""
    return "".join(chr(((value >> (6 * (n - 1 - i))) & 0x3F) + 0x30) for i in range(n))


def split_checked(line: str) -> tuple[str, bool]:
    """Řádek = data + 1 znak součtu. Vrací (data, součet_sedí)."""
    if len(line) < 2:
        return line, False
    return line[:-1], checksum(line[:-1]) == line[-1]


@dataclass
class Stats:
    """Počítadla pro metadata.json."""
    checksum_errors: int = 0      # všechny řádky s chybným součtem
    unknown_status: int = 0
    retries: int = 0
    skipped_replies: int = 0


# =============================================================================
# Komunikace
# =============================================================================
class ScipLink:
    """Čte odpovědi zakončené prázdným řádkem (LF LF) a všechny je ukládá do self.raw."""

    def __init__(self, ser, reply_timeout: float = REPLY_TIMEOUT_S):
        self.ser = ser
        self.reply_timeout = reply_timeout
        self.buf = bytearray()
        self.raw: list[str] = []

    def send(self, cmd: str) -> None:
        log.debug("-> %s", cmd)
        self.ser.write((cmd + "\n").encode("ascii"))

    def read_reply(self, timeout: float | None = None) -> str:
        deadline = time.monotonic() + (timeout or self.reply_timeout)
        while True:
            while self.buf[:1] == b"\n":          # zbytky po předchozí odpovědi
                del self.buf[:1]
            i = self.buf.find(b"\n\n")
            if i >= 0:
                rep = bytes(self.buf[:i + 2]).decode("ascii", errors="replace")
                del self.buf[:i + 2]
                self.raw.append(rep)
                return rep
            if time.monotonic() > deadline:
                raise TimeoutError(f"Lidar neposlal celou odpověď; přijato: {bytes(self.buf[:80])!r}")
            self.buf += self.ser.read(max(1, self.ser.in_waiting))

    def command(self, cmd: str, timeout: float | None = None, other: list[str] | None = None,
                stats: Stats | None = None, max_skip: int = 200) -> str:
        """Pošle příkaz a vrátí odpověď, jejíž echo je přesně cmd.
        Cizí odpovědi (např. dobíhající data MD) se přeskočí; pokud je dán
        seznam `other`, uloží se do něj."""
        self.send(cmd)
        for _ in range(max_skip):
            rep = self.read_reply(timeout)
            if rep.split("\n", 1)[0] == cmd:
                return rep
            if other is not None:
                other.append(rep)
            else:
                log.info("přeskočena odpověď nepatřící k %s: %r", cmd, rep[:30])
                if stats:
                    stats.skipped_replies += 1
        raise RuntimeError(f"Na příkaz {cmd} nepřišla odpověď se správným echem")


def reply_status(reply: str) -> tuple[str, bool]:
    lines = reply.strip("\n").split("\n")
    if len(lines) < 2:
        return "", False
    return split_checked(lines[1])


def parse_info(reply: str, stats: Stats | None = None) -> dict:
    """Odpověď VV/PP/II: řádky 'KLÍČ:HODNOTA;S'. Vrací {'raw':…, 'status':…, 'values':{…}}."""
    lines = reply.strip("\n").split("\n")
    status, ok = split_checked(lines[1]) if len(lines) > 1 else ("", False)
    if not ok:
        log.warning("%s: chybný součet stavového řádku", lines[0])
        if stats:
            stats.checksum_errors += 1
    values = {}
    for line in lines[2:]:
        body, s = line[:-1], line[-1:]
        if not body.endswith(";") or ":" not in body:
            log.warning("%s: neočekávaný formát řádku %r – zahozen", lines[0], line)
            continue
        body = body[:-1]
        primary = body if INFO_SUM_EXCLUDES_SEMICOLON else body + ";"
        alternate = body + ";" if INFO_SUM_EXCLUDES_SEMICOLON else body
        if checksum(primary) != s:
            if checksum(alternate) == s:
                log.warning("%s: řádek %r sedí jen s alternativním výpočtem součtu "
                            "(INFO_SUM_EXCLUDES_SEMICOLON) – přijat", lines[0], line)
            else:
                log.warning("%s: chybný součet řádku %r – zahozen", lines[0], line)
                if stats:
                    stats.checksum_errors += 1
                continue
        k, v = body.split(":", 1)
        values[k] = v
    return {"raw": reply, "status": status, "values": values}


@dataclass
class Params:
    """Parametry senzoru – vždy z odpovědi PP, nikdy natvrdo."""
    DMIN: int
    DMAX: int
    ARES: int
    AMIN: int
    AMAX: int
    AFRT: int
    SCAN: int

    @classmethod
    def from_pp(cls, values: dict) -> "Params":
        missing = [k for k in PP_KEYS if k not in values]
        if missing:
            raise RuntimeError(f"Odpověď PP neobsahuje {missing} – nelze pokračovat bez parametrů senzoru")
        return cls(**{k: int(re.match(r"\s*(\d+)", values[k]).group(1)) for k in PP_KEYS})


def open_port(port: str):
    import serial  # pyserial; funguje pro COM3 i /dev/ttyACM0
    return serial.Serial(port, BAUD, timeout=READ_TIMEOUT_S)


def init_sensor(link: ScipLink, stats: Stats) -> dict:
    """QT, SCIP2.0, VV, PP, II. Vrací slovník pro metadata."""
    try:  # QT zastaví případné běžící MD; ve SCIP 1.1 nemusí odpovědět
        link.command("QT", timeout=1.0, other=[])
    except (TimeoutError, RuntimeError) as e:
        log.warning("QT na začátku bez odpovědi: %s", e)
    time.sleep(0.05)
    link.ser.reset_input_buffer()
    link.buf.clear()
    rep = link.command("SCIP2.0", stats=stats)
    st, _ = reply_status(rep)
    # Kód odpovědi na SCIP2.0 (00 / jiný, když už senzor ve SCIP 2.0 je) – nejisté, jen logujeme.
    log.info("SCIP2.0 -> stav %r", st)
    info = {"SCIP2.0_status": st}
    for cmd in ("VV", "PP", "II"):
        info[cmd] = parse_info(link.command(cmd, stats=stats), stats)
    return info


def laser_on(link: ScipLink, stats: Stats) -> None:
    st, ok = reply_status(link.command("BM", stats=stats))
    if not ok or st not in BM_OK:
        raise RuntimeError(f"BM (zapnutí laseru) vrátil stav {st!r}")


def build_command(kind: str, start: int, end: int, cluster: int, count: int | None = None) -> str:
    cmd = f"{kind}{start:04d}{end:04d}{cluster:02d}"
    if kind in ("MD", "MS"):
        cmd += f"0{count:02d}"    # interval 0 = každý sken; count 00 = nekonečně do QT
    return cmd


# =============================================================================
# Parsování měření
# =============================================================================
@dataclass
class Scan:
    echo: str
    command: str
    start: int
    end: int
    cluster: int
    status: str = ""
    timestamp_ms: float = math.nan
    values: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.int64))
    valid: bool = True
    dropped_lines: int = 0
    problems: list[str] = field(default_factory=list)

    @property
    def expected_count(self) -> int:
        return -(-(self.end - self.start + 1) // self.cluster)   # zaokrouhleno nahoru

    def invalidate(self, msg: str) -> None:
        self.valid = False
        self.problems.append(msg)
        log.warning("%s: %s", self.echo, msg)


def parse_measurement(block: str, stats: Stats | None = None) -> Scan | None:
    """Jedna odpověď GD/GS/MD/MS. Vrací None u potvrzení MD/MS (bez dat)."""
    stats = stats or Stats()
    lines = block.strip("\n").split("\n")
    m = ECHO_GD.match(lines[0]) or ECHO_MD.match(lines[0])
    if not m:
        raise ValueError(f"Není odpověď GD/GS/MD/MS (SCIP 2.0): {lines[0]!r}")
    cmd, start, end, cluster = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4))
    if cluster == 0 and CLUSTER_ZERO_AS_ONE:
        cluster = 1
    scan = Scan(lines[0], cmd, start, end, cluster)

    if len(lines) < 2:
        scan.invalidate("chybí stavový řádek")
        return scan
    status, ok = split_checked(lines[1])
    scan.status = status
    if not ok:
        stats.checksum_errors += 1
        scan.invalidate(f"chybný součet stavového řádku {lines[1]!r}")
        return scan
    if status in STATUS_ACK.get(cmd, ()):
        return None
    if status not in STATUS_DATA[cmd]:
        stats.unknown_status += 1
        scan.invalidate(f"stavový kód {status!r} (není datová odpověď; význam ověřte ve specifikaci)")
        return scan

    if len(lines) < 3:
        scan.invalidate("chybí časová známka")
        return scan
    ts, ok = split_checked(lines[2])
    if ok and len(ts) == 4:
        scan.timestamp_ms = float(decode(ts))
    else:
        stats.checksum_errors += int(not ok)
        scan.invalidate(f"chybná časová známka {lines[2]!r}")

    enc = CHARS_PER_VALUE[cmd]
    parts = []
    for i, line in enumerate(lines[3:], start=1):
        payload, ok = split_checked(line)
        if not ok:
            stats.checksum_errors += 1
            scan.dropped_lines += 1
            scan.invalidate(f"datový řádek {i}: chybný součet – řádek zahozen ({line!r})")
            continue
        if len(payload) > MAX_DATA_LINE:
            scan.invalidate(f"datový řádek {i}: {len(payload)} znaků > {MAX_DATA_LINE}")
        parts.append(payload)
    joined = "".join(parts)          # nejdřív spojit, pak dekódovat (hodnoty přes hranici řádku)
    if len(joined) % enc:
        scan.invalidate(f"počet datových znaků {len(joined)} není násobkem {enc}")
    n = len(joined) // enc
    scan.values = np.array([decode(joined[k * enc:(k + 1) * enc]) for k in range(n)], dtype=np.int64)
    if n != scan.expected_count:
        scan.invalidate(f"dekódováno {n} hodnot, očekáváno {scan.expected_count}")
    return scan


# =============================================================================
# Měření
# =============================================================================
def measure_single(link: ScipLink, cmd: str, retries: int, stats: Stats) -> Scan:
    """Jeden GD/GS sken; při nevalidním skenu až `retries` opakování (vše v log.txt)."""
    for attempt in range(retries + 1):
        scan = parse_measurement(link.command(cmd, stats=stats), stats)
        if scan.valid:
            return scan
        if attempt < retries:
            stats.retries += 1
            log.warning("sken nevalidní, opakuji (%d/%d)", attempt + 1, retries)
    return scan   # poslední (nevalidní) pokus se uloží s valid = 0


def measure_time(link: ScipLink, cmd: str, seconds: float, scans: list, stats: Stats) -> None:
    """Souvislé MD/MS po dobu `seconds`, pak QT. Dobíhající skeny po QT se také uloží."""
    st, ok = reply_status(link.command(cmd, stats=stats))
    if not ok or st not in STATUS_ACK[cmd[:2]]:
        raise RuntimeError(f"{cmd[:2]} nepotvrzen, stav {st!r}")
    t_end = time.monotonic() + seconds
    try:
        while time.monotonic() < t_end:
            s = parse_measurement(link.read_reply(), stats)
            if s is not None:
                scans.append(s)
    finally:
        tail: list[str] = []
        try:
            link.command("QT", other=tail)
        finally:
            for rep in tail:
                if ECHO_MD.match(rep.split("\n", 1)[0]):
                    s = parse_measurement(rep, stats)
                    if s is not None:
                        scans.append(s)


# =============================================================================
# Převod na čísla
# =============================================================================
def step_to_angle_deg(step, p: Params):
    """úhel = (krok − AFRT) · 360 / ARES; kladný doleva (proti směru hodinových ručiček)."""
    return (np.asarray(step, dtype=float) - p.AFRT) * 360.0 / p.ARES


def polar_to_xy(r, angle_deg):
    """x = −r·sin θ (doprava), y = r·cos θ (dopředu), mm."""
    th = np.radians(angle_deg)
    return -r * np.sin(th), r * np.cos(th)


def scan_to_columns(scan: Scan, p: Params, scan_id: int) -> dict[str, np.ndarray]:
    n = scan.expected_count
    step = scan.start + np.arange(n) * scan.cluster
    step_last = np.minimum(step + scan.cluster - 1, scan.end)
    angle = step_to_angle_deg((step + step_last) / 2, p)   # střed skupiny (u cluster 1 = krok)
    # Po zahození řádku nebo nesouladu počtu nejde hodnoty spolehlivě přiřadit krokům -> -1.
    trusted = scan.dropped_lines == 0 and len(scan.values) == n
    raw = scan.values.copy() if trusted else np.full(n, -1, dtype=np.int64)
    valid = scan.valid & (raw >= p.DMIN) & (raw <= p.DMAX)
    dist = np.where(valid, raw, np.nan).astype(float)   # NEmažeme, NEinterpolujeme
    x, y = polar_to_xy(dist, angle)
    return {
        "scan_id": np.full(n, scan_id, dtype=np.int64),
        "timestamp_ms": np.full(n, scan.timestamp_ms, dtype=float),
        "step": step.astype(np.int64),
        "angle_deg": angle,
        "distance_mm": dist,
        "valid": valid.astype(np.int8),
        "x_mm": x,
        "y_mm": y,
        "step_last": step_last.astype(np.int64),
        "raw_value": raw,
        "scan_valid": np.full(n, int(scan.valid), dtype=np.int8),
    }


def scans_to_columns(scans: list[Scan], p: Params) -> dict[str, np.ndarray]:
    parts = [scan_to_columns(s, p, i) for i, s in enumerate(scans)]
    if not parts:
        return {c: np.zeros(0) for c in CSV_COLUMNS}
    return {c: np.concatenate([d[c] for d in parts]) for c in CSV_COLUMNS}


# =============================================================================
# Ukládání
# =============================================================================
def _fmt(v) -> str:
    if isinstance(v, (float, np.floating)):
        return "NaN" if math.isnan(v) else repr(float(v))   # plná přesnost, žádné zaokrouhlení
    return str(int(v))


def write_csv(cols: dict, path: Path) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(CSV_COLUMNS)
        arrays = [cols[c] for c in CSV_COLUMNS]
        for i in range(len(arrays[0])):
            w.writerow([_fmt(a[i]) for a in arrays])


def write_outputs(out: Path, cols: dict, meta: dict, raw: list[str] | None, plot: bool) -> None:
    write_csv(cols, out / "scans.csv")
    np.savez(out / "scans.npz", **cols)
    with open(out / "metadata.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    if raw is not None:
        with open(out / "raw.txt", "w", encoding="ascii", newline="\n") as f:
            f.write("".join(raw))
    if plot:
        save_plot(cols, out / "plot.png")


def save_plot(cols: dict, path: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        log.warning("matplotlib není nainstalován – graf se nevytvoří")
        return
    v = cols["valid"] == 1
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(cols["x_mm"][v], cols["y_mm"][v], s=2)
    ax.plot(0, 0, "r+")
    ax.set_aspect("equal")
    ax.set_xlabel("x [mm] (doprava)")
    ax.set_ylabel("y [mm] (dopředu)")
    ax.grid(True)
    fig.savefig(path, dpi=120)
    plt.close(fig)


def summarize(scans: list[Scan], cols: dict, stats: Stats) -> dict:
    v = cols["valid"] == 1
    d = cols["distance_mm"][v]
    codes: dict[str, int] = {}
    for r in cols["raw_value"][(~v) & (cols["scan_valid"] == 1)]:
        key = f"{int(r)}" + (f" ({ERROR_CODES[int(r)]})" if int(r) in ERROR_CODES else "")
        codes[key] = codes.get(key, 0) + 1
    count_ok = [len(s.values) == s.expected_count for s in scans]
    return {
        "scans_total": len(scans),
        "scans_valid": sum(s.valid for s in scans),
        "scans_invalid": sum(not s.valid for s in scans),
        "count_check_passed": sum(count_ok),
        "points_total": int(len(v)),
        "points_valid": int(v.sum()),
        "points_invalid": int((~v).sum()),
        "distance_min_mm": float(d.min()) if len(d) else None,
        "distance_max_mm": float(d.max()) if len(d) else None,
        "distance_mean_mm": float(d.mean()) if len(d) else None,
        "invalid_raw_values_in_valid_scans": codes,
        **asdict(stats),
    }


def print_summary(s: dict) -> None:
    print(f"skenů: {s['scans_total']} (validních {s['scans_valid']}, nevalidních {s['scans_invalid']}), "
          f"kontrola počtu hodnot OK: {s['count_check_passed']}/{s['scans_total']}")
    print(f"bodů: {s['points_total']} (platných {s['points_valid']}, neplatných {s['points_invalid']})")
    if s["distance_min_mm"] is not None:
        print(f"vzdálenost min/max/průměr: {s['distance_min_mm']:.0f} / {s['distance_max_mm']:.0f} / "
              f"{s['distance_mean_mm']:.1f} mm")
    print(f"chybné součty: {s['checksum_errors']}, neznámé stavy: {s['unknown_status']}, "
          f"opakování: {s['retries']}")
    for k, n in s["invalid_raw_values_in_valid_scans"].items():
        print(f"  hodnota {k}: {n}×")


# =============================================================================
# Hlavní běhy
# =============================================================================
def setup_logging(out: Path) -> list[logging.Handler]:
    out.mkdir(parents=True, exist_ok=True)
    fh = logging.FileHandler(out / "log.txt", mode="w", encoding="utf-8")
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    sh = logging.StreamHandler(sys.stderr)
    sh.setLevel(logging.WARNING)
    sh.setFormatter(logging.Formatter("%(levelname)s %(message)s"))
    log.setLevel(logging.DEBUG)
    for h in (fh, sh):
        log.addHandler(h)
    return [fh, sh]


def run_live(args, ser) -> int:
    """Připojení -> měření -> QT a zavření portu (vždy) -> uložení."""
    out = Path(args.out)
    stats = Stats()
    link = ScipLink(ser)
    scans: list[Scan] = []
    meta: dict = {"source": "simulator" if args.sim else f"port:{args.port}", "mode": args.mode,
                  "platform": platform.platform(), "python": sys.version.split()[0],
                  "time_start_utc": datetime.now(timezone.utc).isoformat(),
                  "interrupted": False, "error": None}
    params = None
    t0 = time.monotonic()
    exit_code = 0
    try:
        info = init_sensor(link, stats)
        meta.update(info)
        params = Params.from_pp(info["PP"]["values"])
        start = args.start if args.start is not None else params.AMIN
        end = args.end if args.end is not None else params.AMAX
        if not (params.AMIN <= start <= end <= params.AMAX):
            log.warning("kroky %d–%d jsou mimo rozsah AMIN–AMAX %d–%d z PP",
                        start, end, params.AMIN, params.AMAX)
        meta.update(start_step=start, end_step=end, cluster=args.cluster,
                    chars_per_value=args.encoding)
        laser_on(link, stats)
        if args.mode == "time":
            cmd = build_command("MD" if args.encoding == 3 else "MS", start, end, args.cluster, 0)
            meta["command"] = cmd
            measure_time(link, cmd, args.seconds, scans, stats)
        else:
            cmd = build_command("GD" if args.encoding == 3 else "GS", start, end, args.cluster)
            meta["command"] = cmd
            for _ in range(1 if args.mode == "single" else args.count):
                scans.append(measure_single(link, cmd, args.retries, stats))
    except KeyboardInterrupt:
        meta["interrupted"] = True
        log.warning("přerušeno uživatelem (Ctrl+C) – ukládám, co bylo naměřeno")
        exit_code = 130
    except Exception as e:  # uložit, co máme, a skončit s chybou
        meta["error"] = f"{type(e).__name__}: {e}"
        log.error("chyba: %s", meta["error"])
        exit_code = 1
    finally:
        try:
            link.command("QT", timeout=1.0, other=[])   # vypnout laser / zastavit MD
        except Exception as e:
            log.warning("závěrečné QT selhalo: %s", e)
        ser.close()
        log.info("port zavřen")
    meta["time_end_utc"] = datetime.now(timezone.utc).isoformat()
    meta["duration_s"] = time.monotonic() - t0
    if params is None:
        meta["summary"] = None
        write_outputs(out, {c: np.zeros(0) for c in CSV_COLUMNS}, meta, link.raw, False)
        return exit_code or 1
    meta["params"] = asdict(params)
    cols = scans_to_columns(scans, params)
    meta["summary"] = summarize(scans, cols, stats)
    write_outputs(out, cols, meta, link.raw, args.plot)
    print_summary(meta["summary"])
    return exit_code


def parse_raw_text(text: str, stats: Stats) -> tuple[list[Scan], dict]:
    """Rozdělí uložený výstup na odpovědi a zpracuje VV/PP/II a GD/GS/MD/MS."""
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    scans, info = [], {}
    for block in re.split(r"\n\n+", text):
        block = block.strip("\n")
        if not block:
            continue
        echo = block.split("\n", 1)[0]
        if echo in ("VV", "PP", "II"):
            info[echo] = parse_info(block + "\n\n", stats)
        elif ECHO_GD.match(echo) or ECHO_MD.match(echo):
            s = parse_measurement(block, stats)
            if s is not None:
                scans.append(s)
        else:
            log.info("odpověď %r přeskočena (není měření ani VV/PP/II)", echo[:20])
    return scans, info


def run_parse(args) -> int:
    out = Path(args.out)
    stats = Stats()
    text = Path(args.parse).read_text(encoding="ascii", errors="replace")
    scans, info = parse_raw_text(text, stats)
    if "PP" in info:
        pp = info["PP"]["values"]
    elif args.meta:
        pp = json.loads(Path(args.meta).read_text(encoding="utf-8"))["PP"]["values"]
    else:
        log.error("v souboru chybí odpověď PP; zadejte --meta metadata.json z původního měření")
        return 2
    params = Params.from_pp(pp)
    cols = scans_to_columns(scans, params)
    meta = {"source": f"parse:{args.parse}", "time_utc": datetime.now(timezone.utc).isoformat(),
            **info, "params": asdict(params),
            "commands": sorted({s.echo[:12] for s in scans}),
            "cluster": sorted({s.cluster for s in scans}),
            "summary": summarize(scans, cols, stats)}
    write_outputs(out, cols, meta, None, args.plot)
    print_summary(meta["summary"])
    return 0


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="COM3 (Windows) nebo /dev/ttyACM0 (Raspberry Pi)")
    src.add_argument("--sim", action="store_true", help="místo lidaru použít simulátor")
    src.add_argument("--parse", metavar="SOUBOR", help="offline dekódovat uložený raw.txt")
    ap.add_argument("--meta", help="metadata.json s odpovědí PP (pro --parse, pokud v souboru chybí)")
    ap.add_argument("--mode", choices=["single", "n", "time"], default="single")
    ap.add_argument("--count", type=int, default=1, help="počet skenů pro --mode n")
    ap.add_argument("--seconds", type=float, default=1.0, help="doba měření pro --mode time")
    ap.add_argument("--encoding", type=int, choices=[3, 2], default=3,
                    help="3 = GD/MD (do 262143 mm), 2 = GS/MS (do 4095 mm)")
    ap.add_argument("--start", type=int, help="počáteční krok (výchozí AMIN z PP)")
    ap.add_argument("--end", type=int, help="koncový krok (výchozí AMAX z PP)")
    ap.add_argument("--cluster", type=int, default=1)
    ap.add_argument("--retries", type=int, default=2, help="opakování nevalidního GD/GS skenu")
    ap.add_argument("--out", required=True, help="výstupní složka")
    ap.add_argument("--plot", action="store_true")
    return ap


def main(argv=None, ser=None) -> int:
    args = build_argparser().parse_args(argv)
    handlers = setup_logging(Path(args.out))
    try:
        if args.parse:
            return run_parse(args)
        if ser is None:
            if args.sim:
                from urg_sim import FakeURG
                ser = FakeURG()
            else:
                ser = open_port(args.port)
        return run_live(args, ser)
    finally:
        for h in handlers:
            log.removeHandler(h)
            h.close()


if __name__ == "__main__":
    sys.exit(main())
