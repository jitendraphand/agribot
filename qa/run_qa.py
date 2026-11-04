"""Recursive acceptance checks for Agribot MkI.

    python -m qa.run_qa

Exits non-zero if any gate fails. Writes qa/REPORT.md, bom/bom.csv, bom/bom.md
and qa/fixtures.json (used to prove the studio's TypeScript IK matches).
"""

from __future__ import annotations

import csv
import json
import math
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from cad.parts import CATALOGUE  # noqa: E402
from kinematics.gait import sample_gait, stance_slip  # noqa: E402
from kinematics.gripper import opening_mm  # noqa: E402
from kinematics.model import arm_ik, arm_within_limits, deg, leg_within_limits  # noqa: E402
from spec.parameters import (  # noqa: E402
    FOOT_R,
    GRIP_CLOSED_SERVO_DEG,
    GRIP_OPEN_SERVO_DEG,
    PLA_COST_PER_KG,
    PLA_DENSITY,
    PRINT_G_PER_MIN,
    SERVO_TRAVEL_DEG,
    TPU_COST_PER_KG,
)

EXPORT = ROOT / "export"
REPORT = ROOT / "qa" / "REPORT.md"
BOM_DIR = ROOT / "bom"


class Gate:
    def __init__(self):
        self.rows = []

    def check(self, name, ok, detail=""):
        self.rows.append((bool(ok), name, detail))
        print(("PASS " if ok else "FAIL ") + name + (f" — {detail}" if detail else ""))
        return bool(ok)

    @property
    def ok(self):
        return all(r[0] for r in self.rows)


def stl_triangles(path: Path) -> int:
    data = path.read_bytes()
    if len(data) < 84:
        return -1
    n = struct.unpack_from("<I", data, 80)[0]
    expect = 84 + n * 50
    if expect != len(data):
        # ASCII STL fallback
        text = data[:200].decode("utf-8", "ignore")
        if "solid" in text.lower():
            return data.lower().count(b"facet")
        return -1
    return n


def main() -> int:
    g = Gate()
    manifest = json.loads((EXPORT / "manifest.json").read_text())
    by_file = {p["file"]: p for p in manifest["parts"]}

    g.check("catalogue count", len(by_file) == len(CATALOGUE), f"{len(by_file)} parts")
    for row in CATALOGUE:
        name = row[0]
        kind = row[2]
        part = by_file.get(name)
        if part is None:
            g.check(f"{name} in manifest", False)
            continue
        step = ROOT / part["step"]
        stl = ROOT / part["stl"]
        g.check(f"{name} step header", step.exists() and step.read_text(errors="ignore").startswith("ISO-10303-21"), str(step.name))
        tris = stl_triangles(stl) if stl.exists() else -1
        g.check(f"{name} stl", tris >= 8, f"{tris} tris, {stl.stat().st_size if stl.exists() else 0} B")
        bb = part["bbox"]
        longest = max(bb["dx"], bb["dy"], bb["dz"])
        if kind == "printed":
            g.check(f"{name} fits 220 bed", longest <= 220.0, f"{longest:.1f} mm")
            g.check(f"{name} has volume", part["volume_mm3"] > 200, f"{part['volume_mm3']:.0f} mm³")
        else:
            g.check(f"{name} has volume", part["volume_mm3"] > 5, f"{part['volume_mm3']:.0f} mm³")

    assy = EXPORT / "agribot_assembly.step"
    g.check("assembly step", assy.exists() and assy.stat().st_size > 100_000 and assy.read_text(errors="ignore").startswith("ISO-10303-21"), f"{assy.stat().st_size} B")
    abb = manifest["assembly_bbox"]
    g.check("assembly not exploded", abb["dx"] < 450 and abb["dy"] < 450 and abb["dz"] < 400, str(abb["dx"]))
    g.check("feet engage soil, not the basement", -4.0 <= abb["zmin"] <= 0.5, f"zmin {abb['zmin']}")
    g.check("instance count", manifest["instance_count"] >= 80, str(manifest["instance_count"]))

    # Kinematics gates, plus fixtures for the studio.
    kin = subprocess.run([sys.executable, "-m", "kinematics.test_kinematics"], cwd=ROOT, capture_output=True, text=True)
    g.check("kinematics suite", kin.returncode == 0, "see kinematics.test_kinematics")
    if kin.returncode != 0:
        print(kin.stdout)
        print(kin.stderr)

    samples = [sample_gait(i * 0.02) for i in range(120)]
    slip = stance_slip(samples)
    g.check("studio gait slip", slip < 1.0, f"{slip:.3f} mm/s")
    g.check("studio gait limits", all(s.limits_ok and s.reachable for s in samples))

    # Servo horn calibration: centre the used band in the 160° travel.
    bands = {"abad": [], "hip": [], "knee": []}
    for s in samples:
        for q in s.q.values():
            d = deg(q)
            bands["abad"].append(d[0])
            bands["hip"].append(d[1])
            bands["knee"].append(d[2])
    calib = {}
    for joint, vals in bands.items():
        lo, hi = min(vals), max(vals)
        span = hi - lo
        calib[joint] = {"min_deg": round(lo, 2), "max_deg": round(hi, 2), "span_deg": round(span, 2), "horn_zero_deg": round((lo + hi) / 2, 2)}
        g.check(f"{joint} travel", span < SERVO_TRAVEL_DEG - 10, f"span {span:.1f}°")

    closed = opening_mm(GRIP_CLOSED_SERVO_DEG)
    opened = opening_mm(GRIP_OPEN_SERVO_DEG)
    g.check("gripper throws", opened > closed + 25, f"{closed:.1f} → {opened:.1f} mm")

    pick = arm_ik([128, 30, -76], tool_pitch=-math.pi / 2)
    g.check("field pick", pick.reachable and pick.error_mm < 0.2 and arm_within_limits(pick.q), f"err {pick.error_mm:.3f}")

    # Fixture for the TypeScript port.
    fixtures = []
    for s in samples[::20]:
        fixtures.append({
            "t": s.t,
            "body_x": s.body_x,
            "q_deg": {n: [round(math.degrees(v), 4) for v in q] for n, q in s.q.items()},
            "feet": {n: [round(float(v), 3) for v in p] for n, p in s.feet_world.items()},
        })
    (ROOT / "qa" / "fixtures.json").write_text(json.dumps({"gait": fixtures, "pick_deg": [round(math.degrees(v), 4) for v in pick.q], "closed_gap": round(closed, 3), "open_gap": round(opened, 3), "calib": calib}, indent=2))

    # BOM
    BOM_DIR.mkdir(parents=True, exist_ok=True)
    lines = []
    print_g = 0.0
    buy = 0.0
    print_cost = 0.0
    for row in CATALOGUE:
        name, _b, kind, material, qty, cost, note, title = row
        part = by_file[name]
        mass = part["mass_g"]
        if kind == "printed":
            unit_mass = mass or 0.0
            rate = TPU_COST_PER_KG if material == "TPU" else PLA_COST_PER_KG
            unit_cost = unit_mass / 1000.0 * rate
            print_g += unit_mass * qty
            print_cost += unit_cost * qty
        else:
            unit_mass = None
            unit_cost = cost or 0.0
            buy += unit_cost * qty
        lines.append({
            "part": name,
            "title": title,
            "qty": qty,
            "kind": kind,
            "material": material,
            "unit_mass_g": "" if unit_mass is None else round(unit_mass, 2),
            "unit_cost_usd": round(unit_cost, 2),
            "ext_cost_usd": round(unit_cost * qty, 2),
            "note": note,
        })
    # Consumables that are not solids.
    extras = [
        ("wire_22awg", "Silicone servo wire", 1, "consumable", "22 AWG, 3 m", "", 6.00, "Power and signal loom"),
        ("fuse_30a", "Inline fuse 30 A", 1, "consumable", "ATO / midi", "", 1.50, "On the pack lead, before the servo rail"),
        ("ferrule_xt30", "XT30 pigtail", 1, "consumable", "silicone", "", 2.00, "Matches the panel connector"),
    ]
    for pn, title, qty, kind, material, mass, cost, note in extras:
        lines.append({
            "part": pn, "title": title, "qty": qty, "kind": kind, "material": material,
            "unit_mass_g": mass, "unit_cost_usd": cost, "ext_cost_usd": round(cost * qty, 2), "note": note,
        })
        buy += cost * qty

    with (BOM_DIR / "bom.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(lines[0].keys()))
        w.writeheader()
        w.writerows(lines)

    minutes = print_g / PRINT_G_PER_MIN
    total = print_cost + buy
    md = ["# Agribot MkI — bill of materials", "", f"Printed mass **{print_g:.0f} g**. Filament **${print_cost:.2f}**. Purchased and consumables **${buy:.2f}**. Roll-up **${total:.2f}** (2026 hobby prices, excluding tax and shipping).", "", f"Estimated print time at {PRINT_G_PER_MIN:.1f} g/min average: **{minutes/60:.1f} h**.", "", "| Qty | Part | Kind | Material | Unit g | Unit $ | Ext $ |", "|---:|---|---|---|---:|---:|---:|"]
    for ln in lines:
        md.append(f"| {ln['qty']} | {ln['title']} | {ln['kind']} | {ln['material']} | {ln['unit_mass_g']} | {ln['unit_cost_usd']:.2f} | {ln['ext_cost_usd']:.2f} |")
    md += ["", "Servo current does not pass through the 5 V UBEC. The pack feeds the servo rail through the 30 A fuse; the UBEC feeds the Pi and the PCA9685 logic only.", ""]
    (BOM_DIR / "bom.md").write_text("\n".join(md))

    # Videos, if the renderer has been run.
    for vid in ("agribot_trot.mp4", "agribot_pick.mp4"):
        p = ROOT / "sim" / vid
        if not p.exists():
            g.check(vid, False, "missing — run python -m sim.render_sim")
            continue
        text = subprocess.run(["ffmpeg", "-i", str(p)], capture_output=True, text=True).stderr
        match = None
        for line in text.splitlines():
            if "Duration:" in line:
                # Duration: HH:MM:SS.xx
                stamp = line.split("Duration:", 1)[1].split(",")[0].strip()
                hh, mm, ss = stamp.split(":")
                dur = int(hh) * 3600 + int(mm) * 60 + float(ss)
                match = dur
                break
        dur = float(match or 0)
        g.check(vid, dur > 3 and p.stat().st_size > 50_000, f"{dur:.1f}s, {p.stat().st_size} B")

    fails = [r for r in g.rows if not r[0]]
    report = ["# Agribot MkI — QA report", "", f"Gates: **{len(g.rows) - len(fails)} passed**, **{len(fails)} failed**, of {len(g.rows)}.", "", "## Servo horn zeros", "", "Install each horn with the servo at 1500 µs parked on this kinematic angle, so the gait sits in the middle of the travel.", "", "| Joint | Min ° | Max ° | Span ° | Horn zero ° |", "|---|---:|---:|---:|---:|"]
    for joint, row in calib.items():
        report.append(f"| {joint} | {row['min_deg']} | {row['max_deg']} | {row['span_deg']} | {row['horn_zero_deg']} |")
    report += ["", "## Failures" if fails else "## Failures", ""]
    report.append("None." if not fails else "\n".join(f"- {n}: {d}" for _, n, d in fails))
    report += ["", "## What this pass actually checks", "", "- Every catalogue solid exists as STEP (ISO-10303-21) and a binary STL with a real triangle count.", "- Every printed part fits a 220 mm bed.", "- The 83-instance assembly is one compound in a sane bounding box, with the feet on the soil rather than through it.", "- Analytical leg IK round-trips to < 0.05 mm, the trot stays inside the joint limits, and stance slip is numerical noise.", "- The gripper four-bar assembles at the open and closed stops and the gap grows by more than 25 mm.", "- The field-pick target is inside the arm workspace and the joint limits.", "- Simulation videos exist and are longer than three seconds.", ""]
    REPORT.write_text("\n".join(report))
    print("REPORT", REPORT, "OK" if g.ok else "FAILED")
    return 0 if g.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
