"""Detailní živý náhled okolí do 1 m z Hokuyo URG-04LX (SCIP 2.0).

Proti původní verzi:
  - používá ověřený parser z urg04lx.py: kontrolní součty, parametry z PP, chybové kódy;
  - GD/MD (3 znaky) místo GS, takže se vzdálenost neořízne na 4095 mm;
  - souvislé měření MD a medián z posledních N skenů pro každý krok (potlačí náhodný šum;
    systematická přesnost lidaru ±10 mm do 1 m podle specifikace zůstává);
  - pohled shora i graf vzdálenost/úhel jen do zvoleného dosahu (výchozí 1000 mm);
  - hledání objektů (souvislé úseky bodů) a tabulka: úhel, vzdálenost, šířka, x/y, šum.

Použití:
    python visualtest.py --list                     # vypíše dostupné COM porty
    python visualtest.py COM6                       # živý graf (mezerník = tabulka objektů, p = uložit PNG)
    python visualtest.py COM6 --once                # medián z 10 skenů, tabulka objektů
    python visualtest.py COM6 --once --points --png okoli.png
    python visualtest.py --sim --once               # bez lidaru (simulátor)

Volby: --range MM (dosah, výchozí 1000), --avg N (počet skenů pro medián, výchozí 10),
       --jump MM (skok vzdálenosti, který odděluje objekty, výchozí 40), --min-points N (výchozí 3),
       --max-width MM (jen objekty užší než MM, např. 60 = kostky, bez stěn),
       --max-noise MM (krok s větším rozptylem mezi skeny se vyřadí jako hrana, výchozí 20).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import warnings
from collections import deque

import numpy as np

import urg04lx as u


# =============================================================================
# Komunikace
# =============================================================================
def open_device(args):
    if args.sim:
        from urg_sim import FakeURG
        return FakeURG()
    return u.open_port(args.port)


def start_stream(link: u.ScipLink, p: u.Params, stats: u.Stats) -> None:
    """Zapne laser a spustí souvislé měření MD (všechny kroky AMIN–AMAX, cluster 1)."""
    u.laser_on(link, stats)
    cmd = u.build_command("MD", p.AMIN, p.AMAX, 1, 0)
    st, ok = u.reply_status(link.command(cmd, stats=stats))
    if not ok or st not in u.STATUS_ACK["MD"]:
        raise RuntimeError(f"MD nepotvrzeno, stav {st!r}")


def read_new_scans(link: u.ScipLink, stats: u.Stats) -> list[u.Scan]:
    """Počká na jeden sken a pak vybere všechny, které už čekají, aby graf nezaostával."""
    replies = [link.read_reply()]
    while link.ser.in_waiting or b"\n\n" in link.buf:
        replies.append(link.read_reply())
    link.raw.clear()   # v živém režimu surová data neukládáme, jinak by rostla paměť
    return [s for s in (u.parse_measurement(r, stats) for r in replies) if s is not None]


# =============================================================================
# Průměrování (medián) přes více skenů
# =============================================================================
class Averager:
    def __init__(self, n: int, p: u.Params, max_noise: float):
        self.p = p
        self.max_noise = max_noise   # [mm] krok s větší směr. odchylkou = nestabilní
        self.unstable = None
        self.med_unstable = None
        self.buf: deque[np.ndarray] = deque(maxlen=n)
        self.rejected = 0
        self.angle = None      # úhel každého kroku [°]
        self.step = None
        self.last = None       # poslední jednotlivý sken (vzdálenosti, NaN = neplatné)

    def add(self, scan: u.Scan) -> None:
        if not scan.valid:     # vadný sken (součet, počet hodnot) se do mediánu nepouští
            self.rejected += 1
            return
        cols = u.scan_to_columns(scan, self.p, 0)
        self.angle, self.step = cols["angle_deg"], cols["step"]
        self.last = cols["distance_mm"]
        self.buf.append(self.last)

    def result(self):
        """Vrací (medián, směrodatná odchylka, počet platných měření) pro každý krok.
        Krok musí být platný aspoň v polovině skenů, jinak NaN (nic se nedopočítává)."""
        stack = np.vstack(self.buf)
        n_ok = np.sum(~np.isnan(stack), axis=0)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)   # sloupce samé NaN
            med = np.nanmedian(stack, axis=0)
            std = np.nanstd(stack, axis=0)
        enough = n_ok >= math.ceil(len(self.buf) / 2)
        med[~enough] = np.nan
        std[~enough] = np.nan
        # Nestabilní krok (velký rozptyl mezi skeny) je typicky hrana předmětu, kde paprsek
        # padá napůl na předmět a napůl na pozadí. Neinterpolujeme, jen vyřadíme (NaN)
        # a zobrazíme zvlášť.
        self.unstable = ~np.isnan(std) & (std > self.max_noise)
        self.med_unstable = np.where(self.unstable, med, np.nan)
        med[self.unstable] = np.nan
        return med, std, n_ok


# =============================================================================
# Hledání objektů do zvoleného dosahu
# =============================================================================
def find_objects(angle, dist, std, rng, jump, min_points, max_width=None):
    """Souvislý úsek = sousední kroky (dovolena mezera 1 neplatného kroku) se skokem
    vzdálenosti nejvýš `jump` mm. Šířka je vzdálenost krajních bodů; je přibližná,
    protože k ní přispívá úhlový krok a průměr paprsku."""
    idx = np.flatnonzero(~np.isnan(dist) & (dist <= rng))
    groups, cur = [], []
    for i in idx:
        if cur and (i - cur[-1] > 2 or abs(dist[i] - dist[cur[-1]]) > jump):
            groups.append(cur)
            cur = []
        cur.append(i)
    if cur:
        groups.append(cur)

    objects = []
    for g in groups:
        if len(g) < min_points:
            continue
        g = np.array(g)
        x, y = u.polar_to_xy(dist[g], angle[g])
        width = math.hypot(x[-1] - x[0], y[-1] - y[0])
        if max_width is not None and width > max_width:
            continue
        m = len(g) // 2   # prostřední bod úseku (u stěny leží těžiště mimo body)
        objects.append({
            "angle_deg": float(angle[g][m]),
            "angle_from": float(angle[g].min()), "angle_to": float(angle[g].max()),
            "dist_center_mm": float(dist[g][m]),
            "dist_min_mm": float(dist[g].min()),
            "width_mm": width,
            "x_mm": float(x[m]), "y_mm": float(y[m]),
            "points": len(g),
            "noise_mm": float(np.nanmedian(std[g])),
            "idx": g,
        })
    return objects


def print_objects(objects, rng, n_scans):
    print(f"\nObjekty do {rng:.0f} mm (medián z {n_scans} skenů):")
    if not objects:
        print("  nic")
        return
    print("   #   úhel°  od°..do°        střed mm  nejbl. mm  šířka mm    x mm    y mm  bodů  šum ±mm")
    for k, o in enumerate(objects, 1):
        print(f"  {k:2d} {o['angle_deg']:+7.1f}  {o['angle_from']:+6.1f}..{o['angle_to']:+6.1f}"
              f"  {o['dist_center_mm']:9.0f}  {o['dist_min_mm']:9.0f}  {o['width_mm']:8.0f}"
              f"  {o['x_mm']:+6.0f}  {o['y_mm']:+6.0f}  {o['points']:4d}  {o['noise_mm']:7.1f}")


def print_points(avg: Averager, med, std, n_ok, rng):
    print(f"\nBody do {rng:.0f} mm (medián):")
    print("  krok    úhel°   vzdál. mm     x mm     y mm   šum ±mm  platných")
    x, y = u.polar_to_xy(med, avg.angle)
    for i in np.flatnonzero(~np.isnan(med) & (med <= rng)):
        print(f"  {avg.step[i]:4d}  {avg.angle[i]:+7.2f}  {med[i]:10.1f}  {x[i]:+7.1f}  {y[i]:+7.1f}"
              f"  {std[i]:8.1f}  {n_ok[i]:3d}/{len(avg.buf)}")


# =============================================================================
# Graf
# =============================================================================
def draw(fig, ax_top, ax_ang, avg: Averager, med, std, objects, rng, p: u.Params, status: str):
    ax_top.cla()
    ax_ang.cla()

    # --- pohled shora: dopředu nahoru, doprava doprava
    for r in np.arange(250, rng + 1, 250):
        ax_top.add_patch(__import__("matplotlib.patches", fromlist=["Circle"]).Circle(
            (0, 0), r, fill=False, lw=0.6, color="0.8"))
        ax_top.text(0, r, f"{r:.0f}", fontsize=7, color="0.5", ha="center", va="bottom")
    for a in (u.step_to_angle_deg(p.AMIN, p), u.step_to_angle_deg(p.AMAX, p)):
        fx, fy = u.polar_to_xy(rng, a)
        ax_top.plot([0, fx], [0, fy], color="0.85", lw=0.8, ls="--")
    if avg.last is not None:
        lx, ly = u.polar_to_xy(avg.last, avg.angle)
        ax_top.scatter(lx, ly, s=3, color="0.7", label="poslední sken")
    mx, my = u.polar_to_xy(med, avg.angle)
    near = med <= rng
    ax_top.scatter(mx[near], my[near], s=8, color="tab:blue", label=f"medián {len(avg.buf)} skenů")
    if avg.med_unstable is not None:
        ux, uy = u.polar_to_xy(avg.med_unstable, avg.angle)
        ax_top.scatter(ux, uy, s=14, marker="x", color="tab:orange", label="nestabilní (hrana)")
    for k, o in enumerate(objects, 1):
        ox, oy = u.polar_to_xy(med[o["idx"]], avg.angle[o["idx"]])
        ax_top.plot(ox, oy, color="tab:red", lw=2)
        ax_top.annotate(f"#{k}\n{o['dist_center_mm']:.0f} mm", (o["x_mm"], o["y_mm"]),
                        textcoords="offset points", xytext=(6, 6), fontsize=8, color="tab:red")
    ax_top.plot(0, 0, marker="^", color="k", ms=8)
    ax_top.set_xlim(-rng * 1.05, rng * 1.05)
    ax_top.set_ylim(-rng * 0.55, rng * 1.05)
    ax_top.set_aspect("equal")
    ax_top.set_xlabel("x [mm]  (doprava →)")
    ax_top.set_ylabel("y [mm]  (dopředu ↑)")
    ax_top.grid(True, lw=0.3)
    ax_top.legend(loc="lower left", fontsize=8)
    ax_top.set_title("Pohled shora")

    # --- vzdálenost podle úhlu; osa otočená, aby levá strana lidaru byla vlevo
    ax_ang.plot(avg.angle, med, color="tab:blue", lw=1, label="medián")
    ax_ang.fill_between(avg.angle, med - std, med + std, color="tab:blue", alpha=0.25, label="± směr. odchylka")
    for k, o in enumerate(objects, 1):
        ax_ang.axvspan(o["angle_from"], o["angle_to"], color="tab:red", alpha=0.08)
        ax_ang.text(o["angle_deg"], o["dist_min_mm"] - rng * 0.04, f"#{k}", color="tab:red",
                    ha="center", va="top", fontsize=8)
    ax_ang.set_xlim(u.step_to_angle_deg(p.AMAX, p), u.step_to_angle_deg(p.AMIN, p))
    ax_ang.set_ylim(0, rng)
    ax_ang.set_xlabel("úhel [°]  (← doleva +, doprava − →)")
    ax_ang.set_ylabel("vzdálenost [mm]")
    ax_ang.grid(True, lw=0.3)
    ax_ang.legend(loc="lower left", fontsize=8)
    ax_ang.set_title("Vzdálenost podle úhlu")

    fig.suptitle(status, fontsize=9)


# =============================================================================
# Hlavní program
# =============================================================================
def list_ports():
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        print(p.device, "-", p.description)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?")
    ap.add_argument("--list", action="store_true", help="vypsat COM porty")
    ap.add_argument("--sim", action="store_true", help="simulátor místo lidaru")
    ap.add_argument("--once", action="store_true", help="jedno měření (medián z --avg skenů) a konec")
    ap.add_argument("--range", type=float, default=1000.0, help="dosah zobrazení a hledání [mm]")
    ap.add_argument("--avg", type=int, default=10, help="počet skenů pro medián")
    ap.add_argument("--jump", type=float, default=40.0, help="skok vzdálenosti oddělující objekty [mm]")
    ap.add_argument("--max-noise", type=float, default=20.0,
                    help="krok se směr. odchylkou nad MM je nestabilní (hrana) a vyřadí se")
    ap.add_argument("--min-points", type=int, default=3, help="min. počet bodů objektu")
    ap.add_argument("--max-width", type=float, help="ukázat jen objekty užší než MM (např. 60 pro kostky)")
    ap.add_argument("--points", action="store_true", help="v režimu --once vypsat i všechny body")
    ap.add_argument("--png", help="uložit graf do souboru (v režimu --once)")
    args = ap.parse_args()

    if args.list or not (args.port or args.sim):
        list_ports()
        return 0

    try:
        ser = open_device(args)
    except Exception as e:   # serial.SerialException
        print(f"Port {args.port} nejde otevřít: {e}\n"
              "Je lidar připojený? Nedrží port jiný program (jiné okno s testem, terminál)?",
              file=sys.stderr)
        return 1
    link = u.ScipLink(ser)
    stats = u.Stats()
    try:
        info = u.init_sensor(link, stats)
        p = u.Params.from_pp(info["PP"]["values"])
        vv = info["VV"]["values"]
        print(f"{vv.get('PROD', '?')}, FW {vv.get('FIRM', '?')}, {vv.get('PROT', '?')}")
        start_stream(link, p, stats)
        avg = Averager(args.avg, p, args.max_noise)

        def status_text():
            return (f"medián z {len(avg.buf)}/{args.avg} skenů · zahozeno vadných skenů {avg.rejected}"
                    f" · chybné součty {stats.checksum_errors} · dosah {args.range:.0f} mm"
                    + (f" · nestabilních kroků {int(avg.unstable.sum())} (šum > {args.max_noise:.0f} mm)"
                       if avg.unstable is not None else ""))

        if args.once:
            while len(avg.buf) < args.avg:
                for s in read_new_scans(link, stats):
                    avg.add(s)
            med, std, n_ok = avg.result()
            objects = find_objects(avg.angle, med, std, args.range, args.jump, args.min_points, args.max_width)
            print(status_text())
            print_objects(objects, args.range, len(avg.buf))
            if args.points:
                print_points(avg, med, std, n_ok, args.range)
            if args.png:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 6.5), gridspec_kw={"width_ratios": [1.2, 1]})
                draw(fig, a1, a2, avg, med, std, objects, args.range, p, status_text())
                fig.tight_layout()
                fig.savefig(args.png, dpi=130)
                print(f"\nGraf uložen: {args.png}")
            return 0

        import matplotlib.pyplot as plt
        plt.ion()
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 6.5), gridspec_kw={"width_ratios": [1.2, 1]})
        state = {}

        def on_key(event):
            if event.key == " " and state:
                print_objects(state["objects"], args.range, len(avg.buf))
            elif event.key == "p":
                name = time.strftime("okoli_%Y%m%d_%H%M%S.png")
                fig.savefig(name, dpi=130)
                print(f"uloženo {name}")

        fig.canvas.mpl_connect("key_press_event", on_key)
        print("Mezerník = vypsat objekty, p = uložit PNG, zavření okna = konec.")
        while plt.fignum_exists(fig.number):
            for s in read_new_scans(link, stats):
                avg.add(s)
            if not avg.buf:
                continue
            med, std, _ = avg.result()
            objects = find_objects(avg.angle, med, std, args.range, args.jump, args.min_points, args.max_width)
            state["objects"] = objects
            draw(fig, a1, a2, avg, med, std, objects, args.range, p, status_text())
            plt.pause(0.001)
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            link.command("QT", timeout=1.0, other=[])   # zastaví MD a vypne laser
        except Exception as e:
            print(f"QT selhalo: {e}", file=sys.stderr)
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
