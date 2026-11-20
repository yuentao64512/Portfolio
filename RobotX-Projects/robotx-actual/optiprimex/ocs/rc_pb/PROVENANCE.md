# rc_pb — RoboNation's RobotX 2026 RoboCommand schema, vendored

- Source: https://github.com/robonation/robocommand, folder `RobotX_2026/`, commit **8fc09f2** (2026-10-24, "Add UAV geofence
  altitude and pilot callsign to RunDeclaration …"). Licence: Apache-2.0 (`LICENSE`, copied unchanged).
- `common_pb2.py`, `robotx/*_pb2.py` (+ `.pyi`): the repo's `gen/python/` output, unchanged (protoc / protobuf runtime 7.35.0 —
  `pip install protobuf==7.35`). `proto/`: the matching `.proto` files, for reference only.
- The generated modules import each other top-level (`import common_pb2`, `from robotx import …`): `__init__.py` puts this
  folder on `sys.path` before importing them. Do not edit the generated files; to update, copy a new `gen/python/` from the
  repo, note the commit here, and run `make test` (tests/test_protobuf_judge.py).
- Handbook 3.4.4: use the competition-provided schema release without changing package names, field numbers, enum values or
  topic names.
