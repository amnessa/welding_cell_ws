# Frozen bench models (test fixture)

The tests replay poses, assemblies and captures recorded on the bench. Those were made
with these CADs. `models/` is the live library and changes; on 2026-10-15 the plate
`test_objv2_base.ply` was replaced by a 220 × 220 × 5.5 mm one, and every test that
placed the old 256 × 249 × 8 mm plate at its recorded pose failed.

- `test_objv2_base.ply`: the old plate, from git (`f0bc2de~1`).
- `test_objv2_ear.ply`, `C1.ply`, `E2.ply`, `R2.ply`, `S3.ply`, `RR1.ply`: copies of
  2026-10-15.
- `weldgen_objects.json`: their registry, built for exactly these files.

Do not refresh this folder from `models/`. A test of the LIVE library belongs on
`models/`.
