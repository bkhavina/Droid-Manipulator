#!/usr/bin/env python3
"""Build and validate the GitHub-ready five-manipulator STL package.

The simulation source meshes use metres.  STL is unitless, but slicers normally
interpret its coordinates as millimetres, so every exported coordinate is scaled
by 1000.  No third-party Python packages are required.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import struct
import sys
from collections import Counter
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
PACKAGE = REPO / "hardware" / "manipulators"
OUT = PACKAGE / "stl"
OBJ_ASSETS = REPO / "simulation" / "mujoco_multi" / "assets"
SO100_ASSETS = REPO / "simulation" / "xlerobot" / "meshes" / "xlerobot" / "assets"
MM_TO_M = 1000.0


def obj_triangles(path: Path):
    vertices = []
    triangles = []
    with path.open("r", encoding="utf-8", errors="strict") as stream:
        for line in stream:
            fields = line.split()
            if not fields:
                continue
            if fields[0] == "v" and len(fields) >= 4:
                vertices.append(tuple(float(value) for value in fields[1:4]))
            elif fields[0] == "f" and len(fields) >= 4:
                face = []
                for token in fields[1:]:
                    index = int(token.split("/", 1)[0])
                    face.append(index - 1 if index > 0 else len(vertices) + index)
                for i in range(1, len(face) - 1):
                    triangles.append((vertices[face[0]], vertices[face[i]], vertices[face[i + 1]]))
    if not vertices or not triangles:
        raise ValueError(f"no mesh data in {path}")
    return triangles


def stl_triangles(path: Path):
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * count:
            triangles = []
            for i in range(count):
                offset = 84 + 50 * i + 12
                triangles.append(tuple(struct.unpack_from("<3f", data, offset + 12 * j) for j in range(3)))
            return triangles

    triangles = []
    current = []
    for line in data.decode("ascii", errors="strict").splitlines():
        fields = line.split()
        if fields[:1] == ["vertex"] and len(fields) == 4:
            current.append(tuple(float(value) for value in fields[1:]))
            if len(current) == 3:
                triangles.append(tuple(current))
                current = []
    if not triangles:
        raise ValueError(f"unsupported or empty STL: {path}")
    return triangles


def scale_triangles(triangles, scale):
    return [tuple(tuple(value * scale for value in vertex) for vertex in tri) for tri in triangles]


def triangle_normal(triangle):
    a, b, c = triangle
    ab = tuple(b[i] - a[i] for i in range(3))
    ac = tuple(c[i] - a[i] for i in range(3))
    normal = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    length = math.sqrt(sum(value * value for value in normal))
    return tuple(value / length for value in normal) if length else (0.0, 0.0, 0.0)


def write_binary_stl(path: Path, triangles):
    path.parent.mkdir(parents=True, exist_ok=True)
    header = b"XLeRobot five-manipulator printable STL (millimetres)"[:80].ljust(80, b"\0")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            values = triangle_normal(triangle) + tuple(value for vertex in triangle for value in vertex)
            stream.write(struct.pack("<12fH", *values, 0))


def mesh_metrics(path: Path):
    triangles = stl_triangles(path)
    points = [vertex for triangle in triangles for vertex in triangle]
    mins = [min(point[axis] for point in points) for axis in range(3)]
    maxs = [max(point[axis] for point in points) for axis in range(3)]
    degenerate = 0
    edges = Counter()
    for triangle in triangles:
        normal = triangle_normal(triangle)
        if normal == (0.0, 0.0, 0.0):
            degenerate += 1
        keys = [tuple(round(value, 5) for value in vertex) for vertex in triangle]
        for a, b in ((keys[0], keys[1]), (keys[1], keys[2]), (keys[2], keys[0])):
            edges[tuple(sorted((a, b)))] += 1
    return {
        "triangles": len(triangles),
        "bbox_min_mm": [round(value, 3) for value in mins],
        "bbox_max_mm": [round(value, 3) for value in maxs],
        "size_mm": [round(maxs[i] - mins[i], 3) for i in range(3)],
        "degenerate_triangles": degenerate,
        "boundary_edges": sum(count == 1 for count in edges.values()),
        "nonmanifold_edges": sum(count > 2 for count in edges.values()),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


PARTS = []


def add_obj(destination, source_name, *, scale=1.0, quantity=1, material="PETG", notes=""):
    PARTS.append(
        dict(
            destination=destination,
            source=OBJ_ASSETS / f"{source_name}.obj",
            source_kind="obj",
            scale=MM_TO_M * scale,
            quantity=quantity,
            material=material,
            notes=notes,
        )
    )


def add_stl(destination, source_name, *, quantity=1, material="PETG", notes=""):
    PARTS.append(
        dict(
            destination=destination,
            source=SO100_ASSETS / f"{source_name}.stl",
            source_kind="stl",
            scale=MM_TO_M,
            quantity=quantity,
            material=material,
            notes=notes,
        )
    )


for name in (
    "Base",
    "Base_Motor",
    "Rotation_Pitch",
    "Rotation_Pitch_Motor",
    "Upper_Arm",
    "Upper_Arm_Motor",
    "Lower_Arm",
    "Lower_Arm_Motor",
    "Wrist_Pitch_Roll",
    "Wrist_Pitch_Roll_Motor",
):
    add_stl(f"common_so100_arm/{name}.stl", name, notes="shared by two_jaw, three_jaw, five_finger, and five_tentacle")

for name in ("Fixed_Jaw", "Fixed_Jaw_Motor", "Moving_Jaw"):
    add_stl(f"two_jaw/{name}.stl", name)

for name, quantity, material in (
    ("tj_housing", 1, "PETG"),
    ("tj_housing_dark", 1, "PETG"),
    ("tj_prox", 3, "PETG"),
    ("tj_dist", 3, "PETG"),
    ("tj_pad", 3, "TPU 95A"),
):
    add_obj(f"three_jaw/{name}.stl", name, quantity=quantity, material=material)

for name in ("hand_palm_p", "hand_palm_n", "hand_palm_dark_p", "hand_palm_dark_n"):
    add_obj(f"five_finger/{name}.stl", name, notes="choose p or n chirality for the robot side")
for name in ("hand_palm_pad_p", "hand_palm_pad_n"):
    add_obj(f"five_finger/{name}.stl", name, material="TPU 95A", notes="choose p or n chirality for the robot side")
for digit in ("index", "middle", "ring", "pinky", "thumb"):
    for segment in range(1, 4):
        name = f"hand_{digit}_{segment}"
        add_obj(f"five_finger/{name}.stl", name)

for name, material in (
    ("tent_mount", "PETG"),
    ("tent_yaw", "PETG"),
    ("tent_base", "PETG"),
    ("tent_base_ring", "PETG"),
):
    add_obj(f"tentacle/{name}.stl", name, material=material)
for index in range(18):
    scale = 0.922**index
    add_obj(f"tentacle/segments/segment_{index:02d}_body.stl", "tent_seg", scale=scale, material="TPU 95A")
    add_obj(f"tentacle/segments/segment_{index:02d}_suckers.stl", "tent_suckers", scale=scale, material="TPU 85A")
add_obj("tentacle/segments/tip_body.stl", "tent_tip", scale=0.922**17, material="TPU 95A")
add_obj("tentacle/segments/tip_suckers.stl", "tent_tip_suckers", scale=0.922**17, material="TPU 85A")

for name, material in (
    ("ft_hub", "PETG"),
    ("ft_hub_orange", "PETG"),
    ("ft_hub_dark", "PETG"),
    ("ft_hub_lens", "clear PETG"),
):
    add_obj(f"five_tentacle/{name}.stl", name, material=material)
for index in range(12):
    scale = 0.93**index
    add_obj(
        f"five_tentacle/segments/segment_{index:02d}_body.stl",
        "ft_seg",
        scale=scale,
        quantity=5,
        material="TPU 95A",
    )
    add_obj(
        f"five_tentacle/segments/segment_{index:02d}_suckers.stl",
        "ft_suckers",
        scale=scale,
        quantity=5,
        material="TPU 85A",
    )
add_obj("five_tentacle/segments/tip_body.stl", "ft_tip", scale=0.93**11, quantity=5, material="TPU 95A")
add_obj(
    "five_tentacle/segments/tip_suckers.stl",
    "ft_tip_suckers",
    scale=0.93**11,
    quantity=5,
    material="TPU 85A",
)

add_obj("vertical_lift/carriage_body.stl", "carriage", material="PETG-CF", notes="shared Z-stage carriage; one per arm")
add_obj(
    "vertical_lift/carriage_rollers_and_accent_reference.stl",
    "carriage_orange",
    material="reference only",
    notes="assembly-position reference for four V-rollers and accent; do not print as one installed part",
)


PREVIEWS = {
    "two_jaw.png": REPO / "simulation" / "mujoco_multi" / "renders" / "robot_two_jaw.png",
    "three_jaw.png": REPO / "simulation" / "mujoco_multi" / "renders" / "robot_three_jaw.png",
    "five_finger.png": REPO / "simulation" / "mujoco_multi" / "renders" / "robot_five_finger.png",
    "tentacle.png": REPO / "simulation" / "mujoco_multi" / "renders" / "robot_tentacle.png",
    "five_tentacle.png": REPO / "simulation" / "mujoco_multi" / "renders" / "robot_five_tentacle.png",
}


def generate():
    manifest = []
    for part in PARTS:
        source = part["source"]
        if not source.is_file():
            raise FileNotFoundError(source)
        triangles = obj_triangles(source) if part["source_kind"] == "obj" else stl_triangles(source)
        destination = OUT / part["destination"]
        write_binary_stl(destination, scale_triangles(triangles, part["scale"]))
        metrics = mesh_metrics(destination)
        entry = {
            "path": destination.relative_to(PACKAGE).as_posix(),
            "source": source.relative_to(REPO).as_posix(),
            "source_to_output_scale": part["scale"],
            "quantity": part["quantity"],
            "material": part["material"],
            "notes": part["notes"],
            **metrics,
        }
        manifest.append(entry)

    preview_dir = PACKAGE / "previews"
    preview_dir.mkdir(parents=True, exist_ok=True)
    for name, source in PREVIEWS.items():
        if source.is_file():
            shutil.copyfile(source, preview_dir / name)

    manifest_path = PACKAGE / "manifest.json"
    manifest_path.write_text(json.dumps({"units": "millimetres", "parts": manifest}, indent=2) + "\n", encoding="utf-8")
    with (PACKAGE / "BOM.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("path", "quantity", "material", "size_mm", "triangles", "notes"))
        for entry in manifest:
            writer.writerow(
                (
                    entry["path"],
                    entry["quantity"],
                    entry["material"],
                    " x ".join(str(value) for value in entry["size_mm"]),
                    entry["triangles"],
                    entry["notes"],
                )
            )
    return validate()


def validate():
    manifest_path = PACKAGE / "manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError("manifest.json is missing; run the exporter without --check first")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    errors = []
    for entry in manifest["parts"]:
        path = PACKAGE / entry["path"]
        if not path.is_file():
            errors.append(f"missing: {entry['path']}")
            continue
        actual = mesh_metrics(path)
        if actual["sha256"] != entry["sha256"]:
            errors.append(f"checksum mismatch: {entry['path']}")
        if actual["triangles"] != entry["triangles"]:
            errors.append(f"triangle-count mismatch: {entry['path']}")
        if actual["degenerate_triangles"]:
            errors.append(f"degenerate triangles ({actual['degenerate_triangles']}): {entry['path']}")
        if not all(value > 0 for value in actual["size_mm"]):
            errors.append(f"zero-size axis: {entry['path']}")
    if errors:
        print("STL validation FAILED", file=sys.stderr)
        print("\n".join(f"- {error}" for error in errors), file=sys.stderr)
        return 1
    total_triangles = sum(entry["triangles"] for entry in manifest["parts"])
    print(f"STL validation passed: {len(manifest['parts'])} files, {total_triangles:,} triangles, millimetre scale")
    shared_edge_files = [entry["path"] for entry in manifest["parts"] if entry["nonmanifold_edges"]]
    if shared_edge_files:
        print(f"Topology note: {len(shared_edge_files)} source-derived files contain coincident/shared edges; see manifest.json")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="validate committed outputs without regenerating them")
    args = parser.parse_args()
    return validate() if args.check else generate()


if __name__ == "__main__":
    raise SystemExit(main())
