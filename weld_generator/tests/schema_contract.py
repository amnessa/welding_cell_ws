"""Contract test for docs/scene.schema.json (schema_version 2.1.0).

Proves the frozen schema accepts the documented example and rejects the
invariants that D12 / D16 / D18 / D19 (D17 withdrawn) depend on. Run standalone:

    python weld_generator/tests/schema_contract.py

Phase 1 wires the same validator over every emitted scene.json in CI.
"""
import json
import pathlib
from jsonschema import Draft202012Validator

SCHEMA_PATH = pathlib.Path(__file__).resolve().parents[1] / "docs" / "scene.schema.json"
S = json.load(open(SCHEMA_PATH))
Draft202012Validator.check_schema(S); print("schema itself: OK")
I4 = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]

def obj(oid, role, obj_id, L,W,t, pgid=None):
    return {"id":oid,"role":role,"object_id":obj_id,"primitive":"slab",
            "dims_mm":[L,W,t],"thickness_mm":t,
            "part_geometry_id":pgid or f"slab_{L}x{W}x{t}","mesh":None,"T_world_part":I4}

def faces_for(oid, start, extra=()):
    out=[{"face_id":start+k,"ref":f"{oid}:{nm}","object":oid,"name":nm,
          "plane":{"n":[0.0,0.0,1.0],"d":-8.0},"surface":None,"area_mm2":100.0}
         for k,nm in enumerate(["+u","-u","+v","-v","+w","-w"])]
    for j,nm in enumerate(extra):
        out.append({"face_id":start+6+j,"ref":f"{oid}:{nm}","object":oid,"name":nm,
                    "plane":None,"surface":{"kind":"cylinder"},"area_mm2":50.0})
    return out

scene={
 "schema_version":"2.1.0","generator_version":"0.1.0",
 "scene_id":"3f9a21c4-0008412337","config_id":"3f9a21c4","seed":8412337,"tier":1,
 "twin_key":"a91cf3e2b7d40518",
 "units":{"length":"mm","angle":"deg"},
 "joint":{"type":"T","seam_shape":"line","quality_level":"C","contact_mode":"flat",
          "included_angle_deg":90.0,"stack_offset_mm":None,"prep":"square"},
 "seam_definition":"nominal",
 "accessibility":{"torch_clearance":{"half_angle_deg":30.0,"standoff_mm":15.0},
                  "dihedral_min_deg":30.0,"dihedral_max_deg":170.0,
                  "contact_tol_mm":3.0,"min_seam_length_mm":10.0},
 "objects":[obj("A","workpiece",0,200.0,120.0,8.0),
            obj("B","workpiece",1,200.0,90.0,8.0),
            obj("F","fixture",255,600.0,400.0,10.0)],
 "faces":faces_for("A",0)+faces_for("B",6)+faces_for("F",12),
 "fit":{"root_gap_mm":1.1,"linear_misalignment_mm":0.4,
        "angular_misalignment_deg":0.8,"throat_thickness_mm":5.6},
 "T_world_joint":I4,
 "seams":[{"id":0,"weldable":True,"reject_reason":None,"face_pair":["A:+w","B:-v"],
           "parametric":{"kind":"line","p0_mm":[0,0,0],"p1_mm":[232,0,0]},
           "length_mm":232.0,"dihedral_deg":90.0,
           "sampled":{"array":"seam_0","density_per_mm":10.0,"n":2321},
           "occluded_fraction":0.34},
          {"id":3,"weldable":False,"reject_reason":"fixture_contact",
           "face_pair":["A:-w","F:+w"],
           "parametric":{"kind":"line","p0_mm":[0,0,0],"p1_mm":[200,0,0]},
           "length_mm":200.0,"dihedral_deg":90.0,
           "sampled":{"array":"seam_3","density_per_mm":10.0,"n":2001},
           "occluded_fraction":0.12}],
 "tacks":None,
 "camera":{"model":"pinhole","K":[[674.0,0,640.0],[0,674.0,360.0],[0,0,1.0]],
           "width":1280,"height":720,"T_world_cam":I4,
           "standoff_mm":750.0,"elevation_deg":55.0},
 "noise_model":{"kind":"stereo_z2","profile":"d435i","subpixel_px":0.08,
                "baseline_mm":50.0,"focal_px":674.0,"min_z_mm":280.0,
                "lateral_sigma_px":0.8,"grazing_dropout_deg":75.0,"seed":8412337},
 "cloud":{"file":"cloud.npz","frame":"world","sampling_mode":"area_uniform",
          "density_per_mm2":1.0,"n_points":184203},
 "rgb":None,"depth":None,
 "provenance":{"created_utc":"2026-08-13T14:31:07Z","git_commit":"9dd4212"}
}
v=Draft202012Validator(S)
def report(name, s, want_valid):
    errs=sorted(v.iter_errors(s), key=lambda e:list(e.path))
    ok = not errs
    good = (ok==want_valid)
    print(f"  {'ok  ' if good else 'FAIL'} : {name}" + ("" if good or ok else f"  -> {errs[0].message[:110]}"))
    return good

print("\npositive cases (must VALIDATE):")
allgood=[report("fixture-present scene, plain slabs, d435i", scene, True)]

# Phase 1 style: no fixture, contact_mode free
p1=json.loads(json.dumps(scene))
p1["objects"]=[o for o in p1["objects"] if o["role"]!="fixture"]
p1["faces"]=[f for f in p1["faces"] if f["object"]!="F"]
p1["joint"]["contact_mode"]="free"
p1["seams"]=[p1["seams"][0]]
allgood.append(report("Phase 1: no fixture + contact_mode 'free'", p1, True))

sg=json.loads(json.dumps(scene))
sg["noise_model"].update(profile="stereo_good",baseline_mm=120.0,focal_px=1100.0,subpixel_px=0.05,min_z_mm=200.0)
sg["camera"]["standoff_mm"]=220.0
allgood.append(report("stereo_good at 220 mm (above its 200 min-Z)", sg, True))

lap=json.loads(json.dumps(scene))
lap["joint"].update(type="lap", included_angle_deg=0, stack_offset_mm=40.0)
allgood.append(report("lap: 0 deg included, stack_offset > 0 (D18)", lap, True))

edge=json.loads(json.dumps(scene))
edge["joint"].update(type="edge", included_angle_deg=0, stack_offset_mm=0)
allgood.append(report("edge: lap at stack_offset = 0 (PARAMETERS 2.7)", edge, True))

print("\nnegative controls (must be REJECTED):")
def neg(name, mut):
    s=json.loads(json.dumps(scene)); mut(s); return report(name, s, False)

allgood += [
 neg("fixture present but contact_mode 'free' (D12)",
     lambda s: s["joint"].__setitem__("contact_mode","free")),
 neg("two fixtures in one scene",
     lambda s: s["objects"].append(obj("G","fixture",254,600.0,400.0,10.0))),
 neg("d435i below its 280 mm min-Z (D16)",
     lambda s: s["camera"].__setitem__("standoff_mm",250.0)),
 neg("missing noise_model.profile (D16)",
     lambda s: s["noise_model"].pop("profile")),
 neg("unknown sensor profile",
     lambda s: s["noise_model"].__setitem__("profile","kinect")),
 neg("a features[] list at all (D17 withdrawn - parts are plain slabs)",
     lambda s: s["objects"][0].__setitem__("features",[])),
 neg("a part_geometry_id carrying a feature hash (D17 withdrawn)",
     lambda s: s["objects"][1].__setitem__("part_geometry_id","slab_200.0x90.0x8.0_f3e91a0c7")),
 neg("a feature face ref (D17 withdrawn - primitive registry only)",
     lambda s: s["seams"][0].__setitem__("face_pair",["A:+w","B:through_hole0"])),
 neg("missing twin_key",
     lambda s: s.pop("twin_key")),
 neg("malformed twin_key",
     lambda s: s.__setitem__("twin_key","NOTHEX")),
 neg("stale schema_version 2.0.0",
     lambda s: s.__setitem__("schema_version","2.0.0")),
]

# no-fixture scene with a non-free contact_mode must also fail
s=json.loads(json.dumps(p1)); s["joint"]["contact_mode"]="flat"
allgood.append(report("no fixture but contact_mode 'flat' (D12)", s, False))

allgood += [
 neg("T-joint included_angle outside ISO 9692-1 60-120 (D18)",
     lambda s: s["joint"].__setitem__("included_angle_deg",45.0)),
 neg("T-joint carrying a stack_offset (D18)",
     lambda s: s["joint"].__setitem__("stack_offset_mm",40.0)),
 neg("missing included_angle_deg (D18)",
     lambda s: s["joint"].pop("included_angle_deg")),
 neg("seam_definition other than 'nominal' (D19)",
     lambda s: s.__setitem__("seam_definition","root")),
 neg("missing seam_definition (D19)",
     lambda s: s.pop("seam_definition")),
 neg("missing accessibility block",
     lambda s: s.pop("accessibility")),
 neg("torch_clearance as a scalar rather than a cone",
     lambda s: s["accessibility"].__setitem__("torch_clearance",15.0)),
 neg("non-square preparation (out of scope, PARAMETERS 5.0)",
     lambda s: s["joint"].__setitem__("prep","single_V")),
 neg("unknown reject_reason",
     lambda s: s["seams"][1].__setitem__("reject_reason","because")),
]

allgood += [
 neg("butt joint with a 90 deg included angle (D18: butt is coplanar, 180)",
     lambda s: s["joint"].update(type="butt", included_angle_deg=90.0)),
 neg("edge joint with a non-zero stack_offset (PARAMETERS 2.7: edge is flush)",
     lambda s: s["joint"].update(type="edge", included_angle_deg=0, stack_offset_mm=25.0)),
]

print("\nALL PASS" if all(allgood) else "\nSOME CHECKS FAILED")
