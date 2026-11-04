"""Build every solid, write STEP + STL, and write the positioned assembly.

    python -m cad.export_all
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from cad.assemble import color_for, instance_list  # noqa: E402
from cad.parts import CATALOGUE, build_all  # noqa: E402
from spec.parameters import PLA_DENSITY, TPU_DENSITY, spec_dict  # noqa: E402

EXPORT = ROOT / "export"
STL = EXPORT / "stl"
STEP = EXPORT / "step"


def _bbox(shape):
    b = shape.BoundingBox()
    return {
        "xmin": round(b.xmin, 3), "ymin": round(b.ymin, 3), "zmin": round(b.zmin, 3),
        "xmax": round(b.xmax, 3), "ymax": round(b.ymax, 3), "zmax": round(b.zmax, 3),
        "dx": round(b.xlen, 3), "dy": round(b.ylen, 3), "dz": round(b.zlen, 3),
    }


def main() -> int:
    STL.mkdir(parents=True, exist_ok=True)
    STEP.mkdir(parents=True, exist_ok=True)
    print("building solids")
    shapes = build_all()
    manifest = {"units": "mm", "parts": [], "assembly": []}
    print("exporting parts")
    for row in CATALOGUE:
        name, _builder, kind, material, qty, cost, note, title = row
        shape = shapes[name]
        vol = float(shape.Volume())  # mm^3
        density = TPU_DENSITY if material == "TPU" else PLA_DENSITY
        mass = vol / 1000.0 * density if kind == "printed" else None
        step_path = STEP / f"{name}.step"
        stl_path = STL / f"{name}.stl"
        print(f"  {name:22s}  {vol:10.0f} mm³")
        cq.exporters.export(shape, str(step_path))
        cq.exporters.export(shape, str(stl_path), tolerance=0.18, angularTolerance=0.28)
        manifest["parts"].append({
            "file": name,
            "title": title,
            "kind": kind,
            "material": material,
            "qty": qty,
            "unit_cost_usd": cost,
            "note": note,
            "volume_mm3": round(vol, 1),
            "mass_g": None if mass is None else round(mass, 2),
            "bbox": _bbox(shape),
            "step": f"export/step/{name}.step",
            "stl": f"export/stl/{name}.stl",
        })

    print("placing assembly")
    items = instance_list(shapes)
    assy = cq.Assembly(name="agribot_mki")
    for instance, shape, M in items:
        placed = shape if False else None
        from cad.assemble import apply
        placed = apply(shape, M)
        assy.add(placed, name=instance, color=color_for(instance))
        manifest["assembly"].append({"instance": instance, "matrix": [[round(float(M[i, j]), 5) for j in range(4)] for i in range(4)]})
    assy_step = EXPORT / "agribot_assembly.step"
    assy_stl = EXPORT / "agribot_assembly.stl"
    print(f"writing {assy_step.name} ({len(items)} instances)")
    assy.save(str(assy_step))
    compound = assy.toCompound()
    print("tessellating assembly")
    cq.exporters.export(compound, str(assy_stl), tolerance=0.35, angularTolerance=0.45)
    manifest["assembly_bbox"] = _bbox(compound)
    manifest["instance_count"] = len(items)
    spec = spec_dict()
    (EXPORT / "manifest.json").write_text(json.dumps(manifest, indent=2))
    (ROOT / "spec" / "robot_spec.json").write_text(json.dumps(spec, indent=2))
    print("done", assy_step, "bbox", manifest["assembly_bbox"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
