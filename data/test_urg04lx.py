"""Unit testy + testy celého programu proti simulátoru (bez lidaru).
Spustit: python -m unittest -v test_urg04lx
Příklady kódování ("CB"=1234, "1Dh"=5432, "m2@0"=16000000) a součty "00"->P, "99"->b
jsou příklady ze specifikace SCIP 2.0 (podle mé paměti, kap. nejisté)."""
import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

import urg04lx as u
from urg_sim import PP, FakeURG, default_scene, scan_reply

P = u.Params.from_pp(PP)


def gd(values, start=44, cluster=1, kind="GD", corrupt_line=None):
    """Sestaví odpověď GD/GS přímo z daných hodnot."""
    scene = np.zeros(769, dtype=np.int64)
    scene[start:start + len(values)] = values
    end = start + len(values) - 1
    echo = u.build_command(kind, start, end, cluster)
    return scan_reply(echo, "00", 1234, scene, start, end, cluster, u.CHARS_PER_VALUE[kind], corrupt_line)


class Decoding(unittest.TestCase):
    def test_spec_examples(self):
        self.assertEqual(u.decode("CB"), 1234)        # 2 znaky
        self.assertEqual(u.decode("1Dh"), 5432)       # 3 znaky
        self.assertEqual(u.decode("m2@0"), 16000000)  # 4 znaky (časová známka)

    def test_roundtrip_limits(self):
        for v in (0, 1, 19, 20, 63, 64, 4095):
            self.assertEqual(u.decode(u.encode(v, 2)), v)
        for v in (0, 20, 4095, 4096, 5600, 262143):
            self.assertEqual(u.decode(u.encode(v, 3)), v)

    def test_checksum_examples(self):
        self.assertEqual(u.checksum("00"), "P")
        self.assertEqual(u.checksum("99"), "b")

    def test_value_split_across_lines(self):
        # 3 znaky/hodnotu, 64 znaků/řádek -> hodnota č. 21 leží na znacích 63..65 (přes hranici)
        vals = [100 + 7 * i for i in range(50)]
        rep = gd(vals)
        data_lines = rep.strip("\n").split("\n")[3:]
        self.assertEqual(len(data_lines[0]), 65)       # 64 dat + součet
        s = u.parse_measurement(rep)
        self.assertTrue(s.valid)
        self.assertEqual(int(s.values[21]), vals[21])
        self.assertEqual(list(s.values), vals)

    def test_split_two_char(self):
        vals = [300 + i for i in range(40)]           # 2 znaky: hranice padne přesně mezi hodnoty
        self.assertEqual(list(u.parse_measurement(gd(vals, kind="GS")).values), vals)


class Checksums(unittest.TestCase):
    def test_bad_data_line_dropped_and_scan_invalid(self):
        stats = u.Stats()
        with self.assertLogs("urg04lx", "WARNING") as cm:
            s = u.parse_measurement(gd([1000] * 682, corrupt_line=2), stats)
        self.assertFalse(s.valid)
        self.assertEqual(s.dropped_lines, 1)
        self.assertEqual(stats.checksum_errors, 1)
        self.assertTrue(any("chybný součet" in m for m in cm.output))
        cols = u.scan_to_columns(s, P, 0)
        self.assertEqual(len(cols["step"]), 682)             # řádky zachované
        self.assertTrue((cols["valid"] == 0).all())
        self.assertTrue(np.isnan(cols["distance_mm"]).all())

    def test_bad_status_and_timestamp(self):
        rep = gd([1000] * 10).split("\n")
        bad_status = "\n".join([rep[0], "00Q", *rep[2:]])
        bad_ts = "\n".join([*rep[:2], rep[2][:-1] + chr(ord(rep[2][-1]) ^ 1), *rep[3:]])
        with self.assertLogs("urg04lx", "WARNING"):
            self.assertFalse(u.parse_measurement(bad_status).valid)
            s = u.parse_measurement(bad_ts)
        self.assertFalse(s.valid)
        self.assertTrue(math.isnan(s.timestamp_ms))

    def test_count_mismatch(self):
        rep = gd([1000] * 10).split("\n")
        echo = u.build_command("GD", 44, 54, 1)           # tvrdí 11 kroků, dat je 10
        with self.assertLogs("urg04lx", "WARNING"):
            s = u.parse_measurement("\n".join([echo, *rep[1:]]))
        self.assertFalse(s.valid)

    def test_info_checksum(self):
        stats = u.Stats()
        rep = "PP\n00P\nDMIN:20;" + u.checksum("DMIN:20") + "\nDMAX:5600;!\n\n"
        with self.assertLogs("urg04lx", "WARNING"):
            info = u.parse_info(rep, stats)
        self.assertEqual(info["values"], {"DMIN": "20"})
        self.assertEqual(stats.checksum_errors, 1)


class Geometry(unittest.TestCase):
    def test_angles(self):
        self.assertEqual(float(u.step_to_angle_deg(P.AFRT, P)), 0.0)
        self.assertAlmostEqual(float(u.step_to_angle_deg(P.AFRT + 256, P)), 90.0)
        self.assertAlmostEqual(float(u.step_to_angle_deg(44, P)), -119.53125)
        self.assertAlmostEqual(float(u.step_to_angle_deg(725, P)), 119.8828125)

    def test_xy(self):
        x, y = u.polar_to_xy(500.0, 0.0)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 500.0)
        x, y = u.polar_to_xy(500.0, 90.0)          # doleva -> záporné x
        self.assertAlmostEqual(x, -500.0)
        self.assertAlmostEqual(y, 0.0, places=9)
        x, _ = u.polar_to_xy(500.0, -90.0)         # doprava -> kladné x
        self.assertAlmostEqual(x, 500.0)

    def test_cluster(self):
        s = u.parse_measurement(gd([300] * 9, start=100, cluster=3, kind="GS"))
        cols = u.scan_to_columns(s, P, 0)
        self.assertEqual(list(cols["step"]), [100, 103, 106])
        self.assertEqual(list(cols["step_last"]), [102, 105, 108])
        s2 = u.parse_measurement(gd([300] * 10, start=100, cluster=3))   # neúplná poslední skupina
        self.assertEqual(s2.expected_count, 4)
        self.assertTrue(s2.valid)


class InvalidValues(unittest.TestCase):
    def test_not_deleted_not_interpolated(self):
        vals = [1000, 0, 19, 20, 5600, 5601, 16, 1000]
        s = u.parse_measurement(gd(vals, start=380))
        cols = u.scan_to_columns(s, P, 0)
        self.assertEqual(len(cols["step"]), len(vals))
        self.assertEqual(list(cols["valid"]), [1, 0, 0, 1, 1, 0, 0, 1])
        self.assertEqual(list(cols["raw_value"]), vals)
        d = cols["distance_mm"]
        self.assertTrue(np.isnan(d[[1, 2, 5, 6]]).all())
        self.assertTrue(np.isnan(cols["x_mm"][1]) and np.isnan(cols["y_mm"][1]))
        self.assertEqual(d[3], 20.0)


REAL = Path(__file__).parent / "testdata" / "real_com6_gd_single.txt"


class RealRecording(unittest.TestCase):
    """Skutečný záznam z URG-04LX (COM6, FW 3.4.03, 2026-11-09), jeden sken GD0044072501."""

    def setUp(self):
        self.stats = u.Stats()
        self.scans, self.info = u.parse_raw_text(REAL.read_text(encoding="ascii"), self.stats)

    def test_info_checksums_all_ok(self):
        self.assertEqual(self.stats.checksum_errors, 0)
        self.assertEqual(self.info["VV"]["values"]["PROT"], "SCIP 2.0")
        p = u.Params.from_pp(self.info["PP"]["values"])
        self.assertEqual((p.DMIN, p.DMAX, p.ARES, p.AMIN, p.AMAX, p.AFRT, p.SCAN),
                         (20, 5600, 1024, 44, 725, 384, 600))

    def test_scan(self):
        self.assertEqual(len(self.scans), 1)
        s = self.scans[0]
        self.assertTrue(s.valid)
        self.assertEqual(s.echo, "GD0044072501")
        self.assertEqual(len(s.values), (725 - 44 + 1) // 1)
        cols = u.scan_to_columns(s, u.Params.from_pp(self.info["PP"]["values"]), 0)
        self.assertEqual(int(cols["valid"].sum()), 537)
        self.assertEqual(set(int(v) for v in cols["raw_value"][cols["valid"] == 0]), {0, 7})
        front = cols["distance_mm"][cols["step"] == 384][0]
        self.assertGreater(front, 20)


class EndToEnd(unittest.TestCase):
    def run_main(self, argv, ser):
        tmp = tempfile.mkdtemp()
        out = Path(tmp) / "out"
        code = u.main(argv + ["--out", str(out)], ser=ser)
        return code, out

    def read_csv(self, out):
        with open(out / "scans.csv", encoding="utf-8") as f:
            return list(csv.DictReader(f))

    def test_single(self):
        sim = FakeURG()
        code, out = self.run_main(["--sim", "--mode", "single"], sim)
        self.assertEqual(code, 0)
        rows = self.read_csv(out)
        self.assertEqual(len(rows), 682)
        self.assertEqual(list(rows[0])[:8], u.CSV_COLUMNS[:8])
        front = next(r for r in rows if r["step"] == "384")
        self.assertEqual(float(front["distance_mm"]), 500.0)
        self.assertEqual(float(front["angle_deg"]), 0.0)
        self.assertEqual(float(front["y_mm"]), 500.0)
        meta = json.loads((out / "metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(meta["params"]["AFRT"], 384)
        self.assertEqual(meta["command"], "GD0044072501")
        self.assertIn("VEND", meta["VV"]["values"])
        self.assertEqual(sim.history[:5], ["QT", "SCIP2.0", "VV", "PP", "II"])
        self.assertEqual(sim.history[-1], "QT")
        self.assertFalse(sim.is_open)
        npz = np.load(out / "scans.npz")
        self.assertEqual(npz["distance_mm"].shape, (682,))
        for name in ("log.txt", "raw.txt"):
            self.assertTrue((out / name).exists())

    def test_n_with_retry(self):
        sim = FakeURG(corrupt={1: 0})          # 2. sken poškozený -> opakování
        code, out = self.run_main(["--sim", "--mode", "n", "--count", "3"], sim)
        meta = json.loads((out / "metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(code, 0)
        self.assertEqual(meta["summary"]["scans_total"], 3)
        self.assertEqual(meta["summary"]["scans_valid"], 3)
        self.assertEqual(meta["summary"]["retries"], 1)
        self.assertIn("opakuji", (out / "log.txt").read_text(encoding="utf-8"))

    def test_time_mode_ms(self):
        sim = FakeURG(scan_period_s=0.05)
        code, out = self.run_main(["--sim", "--mode", "time", "--seconds", "0.3", "--encoding", "2"], sim)
        meta = json.loads((out / "metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(code, 0)
        self.assertEqual(meta["command"], "MS0044072501000")
        n = meta["summary"]["scans_total"]
        self.assertGreaterEqual(n, 3)
        self.assertEqual(meta["summary"]["scans_valid"], n)
        self.assertIsNone(sim.md)               # MD zastaveno
        rows = self.read_csv(out)
        ts = sorted({float(r["timestamp_ms"]) for r in rows})
        self.assertEqual(len(ts), n)            # každý sken vlastní časová známka

    def test_ctrl_c_saves_and_closes(self):
        sim = FakeURG(scan_period_s=0.02, interrupt_after_scans=3)
        code, out = self.run_main(["--sim", "--mode", "time", "--seconds", "5"], sim)
        self.assertEqual(code, 130)
        self.assertEqual(sim.history[-1], "QT")
        self.assertFalse(sim.is_open)
        meta = json.loads((out / "metadata.json").read_text(encoding="utf-8"))
        self.assertTrue(meta["interrupted"])
        self.assertEqual(meta["summary"]["scans_total"], 3)

    def test_timeout_closes_port(self):
        sim = FakeURG(silent=True, timeout=0.01)
        old = u.REPLY_TIMEOUT_S
        try:
            u.ScipLink.__init__.__defaults__ = (0.2,)
            code, out = self.run_main(["--sim"], sim)
        finally:
            u.ScipLink.__init__.__defaults__ = (old,)
        self.assertEqual(code, 1)
        self.assertFalse(sim.is_open)
        meta = json.loads((out / "metadata.json").read_text(encoding="utf-8"))
        self.assertIn("TimeoutError", meta["error"])

    def test_parse_reproduces_live(self):
        code, out = self.run_main(["--sim", "--mode", "n", "--count", "2"], FakeURG())
        code2, out2 = self.run_main(["--parse", str(out / "raw.txt")], None)
        self.assertEqual(code2, 0)
        self.assertEqual((out / "scans.csv").read_text(), (out2 / "scans.csv").read_text())

    def test_parse_crlf(self):
        code, out = self.run_main(["--sim"], FakeURG())
        crlf = Path(out) / "raw_crlf.txt"
        crlf.write_bytes((out / "raw.txt").read_bytes().replace(b"\n", b"\r\n"))
        code2, out2 = self.run_main(["--parse", str(crlf)], None)
        self.assertEqual((out / "scans.csv").read_text(), (out2 / "scans.csv").read_text())


if __name__ == "__main__":
    unittest.main()
