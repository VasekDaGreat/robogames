# Lidar Hokuyo URG-04LX: měření a převod na čísla

Program se připojí k lidaru Hokuyo URG-04LX přes USB (protokol SCIP 2.0) a změří jeden nebo víc skenů.
Každý bod uloží jako vzdálenost v mm, úhel ve stupních a souřadnice x/y v mm.
Funguje na Windows (COM porty) i na Raspberry Pi (`/dev/ttyACM0`) bez úprav kódu.
Bez lidaru se dá vyzkoušet na vestavěném simulátoru.

## Soubory

| Soubor | K čemu je |
|---|---|
| `urg04lx.py` | hlavní program (komunikace, parsování, převod, ukládání) |
| `urg_sim.py` | simulátor lidaru, tedy falešný sériový port pro testy bez hardwaru |
| `test_urg04lx.py` | unit testy a testy celého programu proti simulátoru |
| `*.pdf` | dokumentace Hokuyo (specifikace URG-04LX, protokol SCIP 1.1) |

## Instalace

Potřebujete Python 3.9 nebo novější.

```
pip install pyserial numpy
pip install matplotlib      # volitelné, jen pro --plot
```

Na Raspberry Pi musí mít uživatel přístup k portu: `sudo usermod -a -G dialout $USER`, pak se odhlaste a znovu přihlaste.

## Jaký port použít

- **Windows:** Správce zařízení → Porty (COM a LPT). Lidar se ukáže jako např. `COM3`.
  Ovladač je přiložený v `URG_USB_Driver_*.zip`. Podle specifikace jde o standardní USB CDC, takže na Windows 10/11 by ovladač neměl být potřeba (neověřeno).
- **Raspberry Pi:** obvykle `/dev/ttyACM0`. Ověříte příkazem `ls /dev/ttyACM*` po připojení lidaru.

## Použití

```
# jeden sken
python urg04lx.py --port COM3 --mode single --out mereni1

# 10 skenů za sebou
python urg04lx.py --port /dev/ttyACM0 --mode n --count 10 --out mereni2

# souvislé měření 5 sekund (cca 10 skenů/s)
python urg04lx.py --port COM3 --mode time --seconds 5 --out mereni3

# vyzkoušení bez lidaru (simulátor)
python urg04lx.py --sim --mode single --out test_sim --plot

# znovu dekódovat starší měření offline
python urg04lx.py --parse mereni1/raw.txt --out mereni1_znovu
```

Program skončí kódem 0 (vše v pořádku), 1 (chyba, uloží se, co se stihlo naměřit) nebo 130 (Ctrl+C, data se také uloží).
Na konci vždy pošle `QT` (vypne laser) a zavře port, i při chybě nebo po Ctrl+C.

### Všechny volby

| Volba | Výchozí | Význam |
|---|---|---|
| `--port P` / `--sim` / `--parse SOUBOR` | povinná jedna z nich | zdroj dat |
| `--mode single\|n\|time` | `single` | jeden sken / N skenů / souvislé měření |
| `--count N` | 1 | počet skenů pro `--mode n` |
| `--seconds T` | 1.0 | doba měření pro `--mode time` |
| `--encoding 3\|2` | 3 | 3 = příkazy GD/MD, 2 = GS/MS (kratší data, ale max. 4095 mm) |
| `--start`, `--end` | AMIN, AMAX z PP (44, 725) | rozsah kroků |
| `--cluster C` | 1 | seskupení C sousedních kroků do jedné hodnoty (minimum skupiny) |
| `--retries R` | 2 | kolikrát zopakovat sken s chybou (jen single/n) |
| `--out SLOŽKA` | povinná | kam uložit výsledky |
| `--meta metadata.json` | | parametry PP pro `--parse`, pokud chybí v surovém souboru |
| `--plot` | | uloží `plot.png` (bodový graf x/y) |

## Výstupy (ve složce `--out`)

| Soubor | Obsah |
|---|---|
| `scans.csv` | jeden řádek na krok (při clusteru 1), všechna čísla v plné přesnosti |
| `scans.npz` | stejná data jako numpy pole: `np.load("scans.npz")["x_mm"]` |
| `metadata.json` | odpovědi VV/PP/II, použitý příkaz, parametry senzoru, časy, souhrn, počty chyb |
| `log.txt` | podrobný záznam: chybné součty, neznámé stavy, opakování |
| `raw.txt` | surové odpovědi lidaru beze změny, vstup pro `--parse` |

### Sloupce `scans.csv`

| Sloupec | Jednotka | Význam |
|---|---|---|
| `scan_id` | | pořadí skenu (od 0) |
| `timestamp_ms` | ms | časová známka lidaru (přetéká po 2^24 ms ≈ 4,66 h) |
| `step` | | číslo kroku (u clusteru > 1 první krok skupiny) |
| `angle_deg` | ° | 0° = přímo vpřed, **kladný = doleva** |
| `distance_mm` | mm | vzdálenost, `NaN` když bod není platný |
| `valid` | 0/1 | 1 = bod lze použít |
| `x_mm` | mm | **doprava** kladné, `x = −r·sin θ` |
| `y_mm` | mm | **dopředu** kladné, `y = r·cos θ` |
| `step_last` | | poslední krok skupiny (u clusteru 1 = `step`) |
| `raw_value` | | surová hodnota z lidaru včetně chybového kódu; −1 = nelze určit (vadný sken) |
| `scan_valid` | 0/1 | 0 = celý sken je vadný (chybný součet, špatný počet hodnot, chybový stav) |

Počátek souřadnic je ve středu otáčení lidaru. Neplatné body se **nemažou ani neinterpolují**, jen dostanou `NaN` a `valid = 0`.

### Načtení v Pythonu

```python
import numpy as np
d = np.load("mereni1/scans.npz")
ok = d["valid"] == 1
x, y = d["x_mm"][ok], d["y_mm"][ok]
blizko = d["distance_mm"][ok] < 1000          # body do 1 m
```

## Kdy je bod neplatný

1. **Hodnota < DMIN (20 mm)** je chybový kód, ne vzdálenost. Význam kódu se zapíše do souhrnu.
   Zdroj tabulky je `URG_SCIP11.pdf`, s. 7/8; že platí i pro SCIP 2.0, je nejisté:

   | Kód | Význam |
   |---|---|
   | 0 | objekt možná ve vzdálenosti 22 m |
   | 1–5 | slabá intenzita odraženého světla |
   | 6 | objekt možná ve vzdálenosti 5,7 m |
   | 7 | chyba v předchozím i následujícím kroku |
   | 9 | stejný krok měl chybu ve dvou posledních skenech |
   | 16 | objekt možná v rozsahu 4096 mm – 5,6 m |
   | 18 | nespecifikováno |
   | 19 | vzdálenost nelze změřit |
   | 8, 10–15, 17 | jiné |

2. **Hodnota > DMAX** (lidar hlásí v `PP`, URG-04LX má 5600 mm).
3. **Celý sken je vadný** (`scan_valid = 0`): chybný kontrolní součet stavového řádku, časové známky nebo datového řádku, počet hodnot neodpovídá rozsahu kroků, nebo lidar vrátil chybový stav. Řádek s chybným součtem se zahodí a zapíše do `log.txt`. V režimu single/n se sken zopakuje (`--retries`).

## Přesnost pro LEGO kostky (ze `Specifications_URG-04LX`)

- Rozsah: 20–5600 mm, **zaručená přesnost 60–4095 mm**.
- Přesnost: **±10 mm v rozsahu 60–1000 mm**, nad 1 m ±1 % měření (bílý papír 70×70 mm).
- Úhlový krok: 0,3516° (360°/1024), 683 kroků, výhled 240°, sken trvá 100 ms.
- Průměr paprsku je pod 20 mm na 2000 mm.

Pro představu: kostka 2×2 (asi 16 mm) na vzdálenost 500 mm zabere zhruba 5 kroků, na 1000 mm zhruba 2–3 kroky.
Chyba ±10 mm je srovnatelná s velikostí kostky, proto pomůže průměrovat víc skenů (`--mode n` nebo `--mode time`).

## Živý detailní náhled do 1 m: `testdalsi.py`

```
python testdalsi.py --list                          # dostupné COM porty
python testdalsi.py COM6                            # živé okno (mezerník = tabulka objektů, p = uložit PNG)
python testdalsi.py COM6 --once --png okoli.png     # jedno měření: tabulka + obrázek
python testdalsi.py COM6 --once --max-width 60      # jen malé objekty (kostky), bez stěn
python testdalsi.py COM6 --once --points            # i všechny jednotlivé body do 1 m
```

- Každý bod je **medián posledních 10 skenů** (`--avg`). Šum na stabilní ploše je tak asi 2–5 mm (změřeno na COM6).
  Systematická přesnost lidaru ±10 mm do 1 m zůstává.
- Kroky s rozptylem nad 20 mm mezi skeny (`--max-noise`) se vyřadí jako **nestabilní** (oranžový ×).
  Jsou to typicky hrany předmětů, kde paprsek padá napůl na předmět a napůl na pozadí.
- Objekt je souvislý úsek bodů do `--range` (1000 mm), kde se sousední body liší nejvýš o `--jump` (40 mm).
  Tabulka uvádí úhel, vzdálenost středu a nejbližšího bodu, šířku, x/y a šum.
  Šířka je jen přibližná, protože k ní přispívá úhlový krok a šířka paprsku.
- Vzdálenosti pod 60 mm jsou mimo zaručenou přesnost lidaru (60–4095 mm).

## Testy (bez lidaru)

```
python -m unittest -v test_urg04lx
```

Testy ověřují dekódování 2 a 3 znaků na příkladech ze specifikace, hodnotu zalomenou přes hranici řádku, detekci chybného součtu, převod krok → úhel → x/y, zpracování neplatných hodnot a cluster.
Proti simulátoru se testují i celé běhy: single, n s opakováním, souvislé měření, Ctrl+C, timeout a shoda `--parse` s živým měřením.
Hlášky `WARNING` a `ERROR` během testů jsou očekávané (testy chyby záměrně vyvolávají).

## Ověřeno na skutečném lidaru (2026-11-09, COM6)

Lidar: URG-04LX (Classic-URG), firmware 3.4.03, protokol SCIP 2.0, PP: DMIN 20, DMAX 5600, ARES 1024, AMIN 44, AMAX 725, AFRT 384, SCAN 600.

- Celý průběh `QT → SCIP2.0 → VV → PP → II → BM → GD → QT` funguje.
- Na `SCIP2.0` lidar odpoví `0E` (už je v režimu SCIP 2.0).
- Kontrolní součet řádků VV/PP/II se počítá ze znaků **před** `;`.
- Datové řádky mají 64 znaků + součet. Sken GD0044072501 dal 682 hodnot a všechny součty seděly.
- `--parse` na uloženém `raw.txt` dává přesně stejné CSV jako živé měření.
- Záznam je uložený v `testdata/real_com6_gd_single.txt` a testy ho používají.
- `--mode n --count 5` (GD) dal 5/5 validních skenů.
- `--mode time --seconds 2` (MD) dal 21 skenů, časové známky jdou po 100–101 ms (odpovídá SCAN 600 ot./min).
- `--encoding 2 --cluster 3` (MS) dal 228 hodnot na sken, tedy ⌈682/3⌉, a všechny součty seděly.
- U MS/GS se v datech objevuje přesně 4095, zatímco MD ve stejné scéně hlásí i víc. Dvouznakové kódování zřejmě vzdálenost ořízne na 4095 (nejisté). Pro měření do 1 m to nevadí, ale výchozí `--encoding 3` (GD/MD) je bezpečnější.

## Co ještě není ověřené na skutečném lidaru

- Stavový kód `02` u `BM` (laser už zapnutý) se považuje za OK. Nejisté.
- Běh na Raspberry Pi.
- Starší firmware (do 2.91a) startuje v SCIP 1.1, SCIP 2.0 umí až od 3.1.00. Tento kus má 3.4.03.
- Čísla kapitol specifikace SCIP 2.0: nemáme ji. Ve složce je jen dokumentace k SCIP 1.1.
  Pokud seženete „Communication Protocol Specification SCIP 2.0“ (C-42-3320B), přidejte ji sem.

Při prvním připojení skutečného lidaru pošlete `metadata.json`, `log.txt` a `raw.txt` z jednoho měření. Podle nich se nejistá místa doladí.
