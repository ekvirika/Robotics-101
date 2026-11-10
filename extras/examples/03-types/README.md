# 03 · types

A port without a type accepts anything. Add a type and dora checks, before anything runs, that sender and receiver agree.

```bash
./run.sh types
```

- **`dataflow.yml`:** counter sends `std/core/v1/UInt64`, printer expects `std/core/v1/UInt64`. Result: `All type annotations OK`
- **`broken.yml`:** printer expects `std/media/v1/Image`. Result: a warning, exit code 0
- **`--strict-types`:** the same warning becomes a failure, exit code 1. Put this in CI so wrong wiring never gets merged
- **No runtime cost:** the check happens in `dora validate` and `dora build`, not while the graph runs
