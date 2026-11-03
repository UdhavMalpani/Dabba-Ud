"""
Slice the single-duct test-rig STLs into Bambu Lab P1S print files (.gcode.3mf) with the
Bambu Studio command-line slicer.

Profiles come from the Bambu Studio install (P1S 0.4 mm nozzle, 0.20 mm Standard, Bambu PLA /
PETG Basic). Their "inherits" chains are flattened here because the CLI needs complete profiles.
Process overrides match the mass model in generate_cad_stl.py: 2 walls, 15 % gyroid infill.

    python src/cad/slice_test_rig.py [--parts fit_band,duct_test_rig] [--bambu "C:/.../bambu-studio.exe"]

Output: outputs/test_rig/gcode/<part>.gcode.3mf -> send from Bambu Studio / Handy, or copy to the
printer's SD card.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import zipfile
from pathlib import Path
from typing import Dict

ROOT = Path(__file__).resolve().parent.parent.parent
RIG_DIR = ROOT / "outputs" / "test_rig"
OUT_DIR = RIG_DIR / "gcode"
BAMBU_EXE = Path(r"C:\Program Files\Bambu Studio\bambu-studio.exe")

MACHINE = "Bambu Lab P1S 0.4 nozzle"
PROCESS = "0.20mm Standard @BBL X1C"          # Bambu's 0.20 Standard preset, P1S-compatible
FILAMENT = {"PLA": "Bambu PLA Basic @BBL P1S 0.4 nozzle", "PETG": "Bambu PETG Basic @BBL X1C"}

# part -> (material, extra process overrides)
PARTS = {
    "fit_band":        ("PETG", {"brim_type": "outer_only", "brim_width": "5"}),
    "duct_test_rig":   ("PETG", {"brim_type": "outer_only", "brim_width": "5"}),
    "stand":           ("PLA",  {"brim_type": "outer_only", "brim_width": "5"}),
    "motor_arm":       ("PETG", {}),
    "inlet_grille":    ("PLA",  {}),
    "grille_test_rig": ("PLA",  {}),
    "open_riser":      ("PLA",  {}),
}
COMMON = {"wall_loops": "2", "sparse_infill_density": "15%", "sparse_infill_pattern": "gyroid",
          "enable_support": "0", "curr_bed_type": "Textured PEI Plate"}   # P1S stock plate


def load_flat(kind_dir: Path, name: str) -> Dict:
    """Load a Bambu system profile by name and merge its inherits chain (child wins)."""
    data = json.loads((kind_dir / f"{name}.json").read_text(encoding="utf-8"))
    parent = data.get("inherits")
    if parent:
        base = load_flat(kind_dir, parent)
        base.update(data)
        data = base
    data.pop("inherits", None)
    return data


def write_profile(data: Dict, overrides: Dict, path: Path) -> Path:
    d = dict(data)
    d.update(overrides)
    # the CLI only accepts flattened presets as "system" ones that name the printer explicitly;
    # anything else fails its printer-compatibility check with exit -17
    d["from"] = "system"
    d["inherits"] = ""
    if d.get("type") != "machine":
        d["compatible_printers"] = [MACHINE]
        d["compatible_printers_condition"] = ""
    path.write_text(json.dumps(d, indent=2), encoding="utf-8")
    return path


def gcode_stats(path: Path) -> Dict:
    """Read print time / filament use from the header of the gcode inside the .3mf."""
    with zipfile.ZipFile(path) as z:
        g = next((n for n in z.namelist() if n.endswith(".gcode")), None)
        if not g:
            return {}
        head = z.read(g)[:6000].decode("utf-8", "ignore").splitlines()
    out = {}
    for ln in head:
        low = ln.lower()
        if "total estimated time:" in low:
            out["time"] = low.split("total estimated time:", 1)[1].strip()
        elif "total filament weight" in low:
            out["filament_g"] = ln.split(":", 1)[1].strip()
    return out


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Slice test-rig STLs for the Bambu P1S")
    ap.add_argument("--parts", default=",".join(PARTS))
    ap.add_argument("--bambu", default=str(BAMBU_EXE))
    args = ap.parse_args(argv)

    exe = Path(args.bambu)
    if not exe.exists():
        sys.exit(f"Bambu Studio not found at {exe} (pass --bambu)")
    prof = exe.parent / "resources" / "profiles" / "BBL"
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    work = OUT_DIR / "_profiles"
    work.mkdir(exist_ok=True)

    machine = write_profile(load_flat(prof / "machine", MACHINE), {}, work / "machine.json")
    base_process = load_flat(prof / "process", PROCESS)
    filaments = {m: write_profile(load_flat(prof / "filament", n), {}, work / f"filament_{m}.json")
                 for m, n in FILAMENT.items()}

    for part in [p.strip() for p in args.parts.split(",") if p.strip()]:
        mat, extra = PARTS[part]
        stl = RIG_DIR / f"{part}.stl"
        if not stl.exists():
            print(f"{part}: {stl.name} missing - run src/cad/generate_test_rig.py first")
            continue
        proc = write_profile(base_process, {**COMMON, **extra}, work / f"process_{part}.json")
        out = OUT_DIR / f"{part}.gcode.3mf"
        cmd = [str(exe), "--arrange", "1", "--orient", "0",
               "--load-settings", f"{machine};{proc}", "--load-filaments", str(filaments[mat]),
               "--slice", "0", "--export-3mf", str(out), str(stl)]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        if r.returncode != 0 or not out.exists():
            tail = "\n".join((r.stdout + r.stderr).strip().splitlines()[-12:])
            print(f"{part}: slicing FAILED (exit {r.returncode})\n{tail}")
            continue
        s = gcode_stats(out)
        print(f"{part:<16} {mat:<5} {s.get('time', '?'):>14}  {s.get('filament_g', '?'):>8} g  -> {out.name}")


if __name__ == "__main__":
    main()
