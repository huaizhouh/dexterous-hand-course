#!/usr/bin/env python3
"""Build a three-tendon / six-passive-joint hand from the supplied CAD parts.

All modelling coordinates and exported STEP/STL dimensions are millimetres.
Run with the dependencies in requirements.txt, from any working directory.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf
from OCP.Interface import Interface_Static
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "support_frames"
PARTS = HERE / "source_step"
GENERATED = HERE / "custom_parts"
FINGERS = [
    ("thumb", (0., -70., 27.5), 180.),
    ("index", (-38., 65., 27.5), 0.),
    ("middle", (38., 65., 27.5), 0.),
]
CABLE_X = 14.
CABLE_Y = 15.
MOUNT_HOLES = [(x, y) for x in (-12., 12.) for y in (-8., 8.)]


def T(x=0., y=0., z=0.):
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def R(axis, degrees):
    a = math.radians(degrees)
    c, s = math.cos(a), math.sin(a)
    m = np.eye(4)
    if axis == "x":
        m[:3, :3] = ((1, 0, 0), (0, c, -s), (0, s, c))
    elif axis == "z":
        m[:3, :3] = ((c, -s, 0), (s, c, 0), (0, 0, 1))
    return m


def transformed(shape, matrix):
    trsf = gp_Trsf()
    trsf.SetValues(*[float(v) for v in matrix[:3, :].flat])
    return shape.moved(cq.Location(trsf))


def export_single_part(shape, target):
    writer = STEPControl_Writer()
    assembly_mode = Interface_Static.IVal_s("write.step.assembly")
    try:
        Interface_Static.SetIVal_s("write.step.assembly", 0)
        Interface_Static.SetCVal_s("write.step.unit", "MM")
        Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
        writer.Transfer(shape.wrapped, STEPControl_AsIs)
        writer.Write(str(target))
    finally:
        Interface_Static.SetIVal_s("write.step.assembly", assembly_mode)
    reread = cq.importers.importStep(str(target)).val()
    text = target.read_text()
    report = dict(products=len(re.findall(r"=\s*PRODUCT\s*\(", text)),
                  assembly_occurrences=len(re.findall(r"=\s*NEXT_ASSEMBLY_USAGE_OCCURRENCE\s*\(", text)),
                  solids=len(reread.Solids()), valid=all(s.isValid() for s in reread.Solids()),
                  volume_delta_mm3=reread.Volume()-shape.Volume(),
                  bounds_delta_mm=[getattr(reread.BoundingBox(), k)-getattr(shape.BoundingBox(), k)
                                   for k in ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax")])
    if (report["products"] != 1 or report["assembly_occurrences"] != 0
        or report["solids"] != len(shape.Solids()) or not report["valid"]
        or abs(report["volume_delta_mm3"]) > .001
        or max(map(abs, report["bounds_delta_mm"])) > .00001):
        raise RuntimeError(f"Single-part STEP validation failed: {report}")
    return report


def point(matrix, p):
    return (matrix @ np.array((*p, 1.)))[:3]


def box(w, d, h, center):
    return cq.Workplane("XY").box(w, d, h).val().translate(center)


def cylinder(radius, length, start, direction=(0, 0, 1)):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), cq.Vector(*direction))


def holes_z(shape, points, radius, z, depth):
    for x, y in points:
        shape = shape.cut(cylinder(radius, depth, (x, y, z)))
    return shape


def cable(points, radius=0.4):
    """Piecewise straight cable with spherical joins, retained as one solid."""
    result = cq.Solid.makeSphere(radius * 1.01, cq.Vector(*points[0]), angleDegrees1=-90)
    for a, b in zip(points, points[1:]):
        a, b = np.asarray(a), np.asarray(b)
        v = b - a
        if np.linalg.norm(v) < 1e-7:
            continue
        segment = cylinder(radius, float(np.linalg.norm(v)), a, v / np.linalg.norm(v))
        result = result.fuse(segment)
        result = result.fuse(cq.Solid.makeSphere(radius * 1.01, cq.Vector(*b), angleDegrees1=-90))
    return result.clean()


def convert_sources():
    sys.path.insert(0, str(HERE / "tools/vendor/sldprt2step"))
    from sldprt2step import run_conversion
    PARTS.mkdir(exist_ok=True)
    reports = []
    for source in sorted(SOURCE.glob("*.SLDPRT")):
        target = PARTS / (source.stem + ".step")
        result = run_conversion(str(source), str(target))
        result.update(source=source.name, sha256=hashlib.sha256(source.read_bytes()).hexdigest())
        reports.append(result)
        if not result["ok"]:
            raise RuntimeError(f"Cannot convert {source.name}: {result}")
    (PARTS / "conversion_report.json").write_text(json.dumps(reports, ensure_ascii=False, indent=2))


def load_sources():
    shapes, reports = {}, []
    for source in sorted(SOURCE.glob("*.SLDPRT")):
        p = PARTS / (source.stem + ".step")
        if not p.exists():
            raise FileNotFoundError(f"{p}: run with --convert")
        s = cq.importers.importStep(str(p)).val()
        b = s.BoundingBox()
        row = dict(name=source.stem, valid=s.isValid(), solids=len(s.Solids()),
                   volume_mm3=s.Volume(), faces=len(s.Faces()),
                   bounds_mm=[b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax],
                   source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
        reports.append(row)
        if not row["valid"] or row["solids"] != 1 or row["volume_mm3"] <= 0:
            raise RuntimeError(f"Invalid converted source: {row}")
        shapes[source.stem] = s
    (PARTS / "geometry_report.json").write_text(json.dumps(reports, ensure_ascii=False, indent=2))
    return shapes


def custom_parts():
    palm = cq.Workplane("XY").box(160, 190, 8, centered=(True, True, False)).edges("|Z").fillet(9).val()
    # Four new mounting holes plus one new tendon bore in each small-U copy.
    mount_holes = MOUNT_HOLES
    for _, position, yaw in FINGERS:
        root = T(*position) @ R("z", yaw)
        riser = box(44, 42, 8, (0, 0, -15.5))  # world z=8..16
        palm = palm.fuse(transformed(riser, root))
        for x, y in mount_holes:
            p = point(root, (x, y, -30))
            palm = palm.cut(cylinder(1.2, 28, p))
        palm = palm.cut(cylinder(1.2, 28, point(root, (CABLE_X, -CABLE_Y, -30))))
    # Wrist attachment and a lightening opening in the empty palm centre.
    palm = holes_z(palm, [(-65, -82), (65, -82), (-65, 82), (65, 82)], 2.7, -1, 10)
    palm = palm.cut(cq.Workplane("XY").rect(38, 32).extrude(10).val().translate((0, 0, -1))).clean()

    adapter = box(52, 42, 4, (0, 0, 54))  # proximal long-U end at z=52
    # Match existing long-U base holes and small-U base holes independently.
    adapter = holes_z(adapter, [(0, -7), (0, 7), (-18.5, 0), (18.5, 0)], 1.6, 51, 6)
    adapter = holes_z(adapter, mount_holes, 1.2, 51, 6)
    adapter = holes_z(adapter, [(CABLE_X, -CABLE_Y)], 1.2, 51, 6)
    adapter = adapter.cut(cylinder(4.1, 6, (0, 0, 51)))

    end_z = 30.01 + 2
    tip = box(52, 42, 4, (0, 0, end_z + 2))
    tip = holes_z(tip, [(-7, 0), (7, 0), (0, -7), (0, 7)], 1.6, end_z - 1, 6)
    tip = holes_z(tip, [(CABLE_X, -CABLE_Y), (-CABLE_X, CABLE_Y)], 1.2, end_z - 1, 6)
    pad = cq.Workplane("XY").box(46, 24, 8).edges("|Z").fillet(3).val().translate((0, 0, end_z + 8))
    # Axle journals fit the original 8 mm bracket holes; axial clearance retained.
    bush = cylinder(3.9, 7.65, (20.3, 0, 0), (1, 0, 0)).cut(cylinder(1.6, 7.85, (20.2, 0, 0), (1, 0, 0)))
    pin = cylinder(1.4, 67, (-33.5, 0, 0), (1, 0, 0))
    # Cable anchor button above the endplate; its small neck fits the cable hole.
    anchor = cylinder(2.8, 2, (CABLE_X, -CABLE_Y, end_z + 4)).fuse(
        cylinder(1.0, 2, (CABLE_X, -CABLE_Y, end_z + 2)))
    return dict(palm=palm, interphalangeal_adapter=adapter.clean(), fingertip_carrier=tip.clean(),
                fingertip_pad=pad, journal_bushing=bush.clean(), hinge_pin=pin, tendon_anchor=anchor.clean())


def assemble(sources, custom, angles, pose):
    assembly = cq.Assembly(name="tendon_three_finger_hand_" + pose)
    records = []
    colors = dict(palm=(0.23, 0.28, 0.34), frame=(0.67, 0.72, 0.78),
                  horn=(0.90, 0.65, 0.21), adapter=(0.21, 0.46, 0.64),
                  pad=(0.15, 0.18, 0.20), axle=(0.34, 0.37, 0.40), cable=(0.95, 0.17, 0.10))

    def add(name, shape, matrix, part, color, source=None):
        shape = transformed(shape, matrix)
        if not shape.isValid() or len(shape.Solids()) != 1:
            raise RuntimeError(f"Invalid assembled part {name}")
        assembly.add(shape, name=name, color=cq.Color(*colors[color]))
        records.append(dict(name=name, shape=shape, part=part, source=source,
                            matrix=matrix.tolist(), color=colors[color]))

    add("palm", custom["palm"], np.eye(4), "palm", "palm")
    joints, routes = [], {}
    # Small-U backplate y=0..1.5; pivot y=-10,z=0, along x.
    small_normalize = R("x", -90) @ T(0, 10, 0)
    long_normalize = R("x", 180) @ T(-27, 12.5, -50)
    short_normalize = R("x", 180) @ T(-27, 12.5, -30.01)
    small_drilled = sources["小U连接件"]
    for x, y in MOUNT_HOLES + [(CABLE_X, -CABLE_Y)]:
        small_drilled = small_drilled.cut(cylinder(1.2, 5, (x, -1, y), (0, 1, 0)))
    modified = HERE / "modified_parts"
    modified.mkdir(exist_ok=True)
    cq.exporters.export(small_drilled, str(modified / "small_u_drilled.step"))
    for finger, origin, yaw in FINGERS:
        root = T(*origin) @ R("z", yaw)
        proximal = root @ R("x", angles[0])
        second = proximal @ T(z=67.5)
        distal = second @ R("x", angles[1])
        add(finger + "_root_small_u", small_drilled, root @ small_normalize,
            "小U连接件_钻孔副本", "frame", "小U连接件.SLDPRT + modified_parts/small_u_drilled.step")
        add(finger + "_proximal_long_u", sources["长U支架"], proximal @ long_normalize,
            "长U支架", "frame", "长U支架.SLDPRT")
        add(finger + "_adapter", custom["interphalangeal_adapter"], proximal,
            "interphalangeal_adapter", "adapter")
        add(finger + "_distal_small_u", small_drilled, second @ small_normalize,
            "小U连接件_钻孔副本", "frame", "小U连接件.SLDPRT + modified_parts/small_u_drilled.step")
        add(finger + "_distal_short_u", sources["短U支架"], distal @ short_normalize,
            "短U支架", "frame", "短U支架.SLDPRT")
        for part, color in [("fingertip_carrier", "adapter"), ("fingertip_pad", "pad"), ("tendon_anchor", "horn")]:
            add(finger + "_" + part, custom[part], distal, part, color)
        for number, fixed, moving in [(1, root, proximal), (2, second, distal)]:
            prefix = f"{finger}_j{number}"
            for side, source, rx in [(1, "金属主舵盘", 0), (-1, "金属从舵盘", 180)]:
                # Put both flanges OUTSIDE the U ears, bosses facing away.
                # Native horn y-axis maps to +/- finger x-axis.
                horn_pose = T(x=28 * side) @ R("z", -90 * side)
                add(prefix + ("_horn_main" if side > 0 else "_horn_follower"), sources[source],
                    moving @ horn_pose, source, "horn", source + ".SLDPRT")
                bush_pose = np.eye(4) if side > 0 else R("z", 180)
                add(prefix + f"_bush_{side:+d}".replace("+", "p").replace("-", "m"),
                    custom["journal_bushing"], fixed @ bush_pose, "journal_bushing", "axle")
            add(prefix + "_pin", custom["hinge_pin"], fixed, "hinge_pin", "axle")
            joints.append(dict(name=prefix, type="passive_revolute",
                               origin_mm=point(fixed, (0, 0, 0)).tolist(), angle_deg=angles[number - 1],
                               axis_direction=(fixed[:3, :3] @ np.array((1, 0, 0))).tolist()))

        # A separate strand per finger: underneath-palm inlet, drilled root
        # backplate hole, drilled second backplate hole, fingertip anchor.
        # At each backplate, a short normal segment keeps the cable in its bore.
        route = [point(root, (CABLE_X, -CABLE_Y, z)) for z in (-36.5, -12, -9.7)]
        route += [point(proximal, (CABLE_X, -CABLE_Y, z)) for z in (53.5, 55.5, 58.2, 62)]
        route += [point(distal, (CABLE_X, -CABLE_Y, z)) for z in (29.5, 31.8, 34.01)]
        tendon = cable(route)
        add(finger + "_flexor_tendon", tendon, np.eye(4), "flexor_tendon", "cable")
        routes[finger] = dict(points_mm=[p.tolist() for p in route],
                              diameter_mm=0.8, length_mm=sum(float(np.linalg.norm(b-a)) for a,b in zip(route,route[1:])))
    return assembly, records, joints, routes


def intersections(records):
    collisions, terminations = [], []
    for i, a in enumerate(records):
        ba = a["shape"].BoundingBox()
        for b in records[i + 1:]:
            bb = b["shape"].BoundingBox()
            if any(min(getattr(ba, k+"max"), getattr(bb, k+"max")) -
                   max(getattr(ba, k+"min"), getattr(bb, k+"min")) < 1e-5 for k in "xyz"):
                continue
            volume = a["shape"].intersect(b["shape"]).Volume()
            if volume > 0.02:
                entry = dict(a=a["name"], b=b["name"], volume_mm3=volume)
                if {a["part"], b["part"]} == {"tendon_anchor", "flexor_tendon"} and a["name"].split("_")[0] == b["name"].split("_")[0]:
                    terminations.append(entry)
                else:
                    collisions.append(entry)
    return collisions, terminations


def validate(records, step_path):
    # Reimport the actual deliverable, independently of the in-memory assembly.
    reread = cq.importers.importStep(str(step_path)).val()
    invalid = [i for i, s in enumerate(reread.Solids()) if not s.isValid() or s.Volume() <= 0]
    collisions, terminations = intersections(records)
    bounds = reread.BoundingBox()
    return dict(step_reimport_valid=not invalid, expected_solids=len(records),
                actual_solids=len(reread.Solids()), invalid_solids=invalid,
                bounding_box_mm=[bounds.xlen, bounds.ylen, bounds.zlen],
                intentional_cable_anchor_embeddings=terminations,
                intersections_over_0_02_mm3=collisions)


def preview(records, output, pose):
    """Render with a depth buffer; requires access to a graphics session."""
    import vtk
    from vtkmodules.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(.94, .96, .98)
    for item in records:
        vs, fs = item["shape"].tessellate(.15, .15)
        coordinates = np.array([v.toTuple() for v in vs])
        faces = np.asarray(fs, dtype=np.int64)
        points = vtk.vtkPoints()
        points.SetData(numpy_to_vtk(coordinates, deep=True))
        cells = vtk.vtkCellArray()
        cells.SetData(numpy_to_vtkIdTypeArray(np.arange(len(faces)+1, dtype=np.int64)*3, deep=True),
                      numpy_to_vtkIdTypeArray(faces.ravel(), deep=True))
        poly = vtk.vtkPolyData()
        poly.SetPoints(points)
        poly.SetPolys(cells)
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(poly)
        normals.ConsistencyOn()
        normals.AutoOrientNormalsOn()
        normals.SetFeatureAngle(35)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(normals.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*item["color"])
        actor.GetProperty().SetAmbient(.3)
        actor.GetProperty().SetDiffuse(.7)
        actor.GetProperty().SetSpecular(.15)
        actor.GetProperty().SetSpecularPower(35)
        renderer.AddActor(actor)
    camera = renderer.GetActiveCamera()
    camera.SetPosition(280, -380, 260)
    camera.SetFocalPoint(0, 0, 60)
    camera.SetViewUp(0, 0, 1)
    camera.ParallelProjectionOn()
    camera.SetParallelScale(145)
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1500, 1300)
    window.SetMultiSamples(4)
    window.AddRenderer(renderer)
    window.Render()
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(output))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()
    window.Finalize()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--convert", action="store_true", help="Regenerate source STEP files from SLDPRT")
    parser.add_argument("--skip-preview", action="store_true")
    args = parser.parse_args()
    if args.convert:
        convert_sources()
    sources = load_sources()
    custom = custom_parts()
    GENERATED.mkdir(exist_ok=True)
    for name, shape in custom.items():
        if not shape.isValid():
            raise RuntimeError("Invalid custom part: " + name)
        cq.exporters.export(shape, str(GENERATED / (name + ".step")))
    report, config, meshes, single_parts = {}, {}, {}, {}
    bom_records = None
    for pose, angles in [("open", (0., 0.)), ("grasp_ready", (18., 32.))]:
        assembly, records, joints, routes = assemble(sources, custom, angles, pose)
        target = HERE / ("tendon_three_finger_hand_" + pose + ".step")
        assembly.export(str(target), exportType="STEP", mode="default")
        compound = cq.Compound.makeCompound([r["shape"] for r in records])
        # A single product with multiple solid bodies avoids creating a part
        # document for every component when viewing the complete hand.
        single_part = HERE / ("tendon_three_finger_hand_" + pose + "_single_part.step")
        single_parts[single_part.name] = export_single_part(compound, single_part)
        cq.exporters.export(compound, str(target.with_suffix(".stl")), tolerance=.15, angularTolerance=.15)
        import trimesh
        mesh = trimesh.load_mesh(target.with_suffix(".stl"))
        meshes[target.with_suffix(".stl").name] = dict(triangles=len(mesh.faces), bounds_mm=mesh.extents.tolist(),
            watertight=bool(mesh.is_watertight), winding_consistent=bool(mesh.is_winding_consistent),
            volume_mm3=float(mesh.volume))
        print("Exported", target.name, len(records), "components", flush=True)
        report[pose] = validate(records, target)
        print(json.dumps(report[pose], ensure_ascii=False), flush=True)
        config[pose] = dict(joints=joints, tendons=routes, instances=[{k:v for k,v in r.items() if k!="shape"} for r in records])
        if not args.skip_preview:
            preview(records, target.with_suffix(".png"), pose)
        if bom_records is None:
            bom_records = records
    with (HERE / "bom.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["part", "quantity", "source", "notes"])
        for part, qty in Counter(r["part"] for r in bom_records).items():
            source = next(r["source"] for r in bom_records if r["part"] == part)
            path = "assembly_manifest.json: tendons" if part == "flexor_tendon" else "custom_parts/" + part + ".step"
            writer.writerow([part, qty, source or path,
                             "diameter 0.8 mm; routing reference" if part=="flexor_tendon" else ""])
    (HERE / "assembly_manifest.json").write_text(json.dumps(config, ensure_ascii=False, indent=2))
    (HERE / "validation_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    (HERE / "mesh_report.json").write_text(json.dumps(meshes, indent=2))
    (HERE / "single_part_validation_report.json").write_text(json.dumps(single_parts, indent=2))
    motion = []
    for fraction in (.125, .25, .375, .5, .625, .75, .875):
        _, records, _, routes = assemble(sources, custom, (18*fraction, 32*fraction), "sweep")
        collisions, _ = intersections(records)
        motion.append(dict(fraction=fraction, angles_deg=[18*fraction, 32*fraction],
                           intersections_over_0_02_mm3=collisions,
                           tendon_lengths_mm={k:v["length_mm"] for k,v in routes.items()}))
    (HERE / "motion_clearance_report.json").write_text(json.dumps(motion, ensure_ascii=False, indent=2))
    print("Motion clearance:", len(motion), "intermediate poses checked", flush=True)
    if any(not r["step_reimport_valid"] or r["actual_solids"] != r["expected_solids"] or r["intersections_over_0_02_mm3"] for r in report.values()):
        raise SystemExit("Validation requires attention; see validation_report.json")
    if any(r["intersections_over_0_02_mm3"] for r in motion):
        raise SystemExit("Motion clearance requires attention; see motion_clearance_report.json")


if __name__ == "__main__":
    main()
