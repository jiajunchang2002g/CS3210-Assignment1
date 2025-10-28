# CS3210 — Scheduling (SCH) Assignment

This repository contains the code and assets for the scheduling (SCH) assignment used in CS3210. The project implements a simulation and benchmarking harness for evaluating scheduling/traffic behavior.

## Repository layout

- `simulation.cc`, `traffic.cc`, `traffic.h`, `common.cc`, `common.h` — core simulation and traffic source code.
- `Makefile` — build recipes.
- `gen.py` — input generator for benchmarks/tests.
- `run_bench.sh` — helper script to run benchmark suites.
- `executables/` — prebuilt example binaries (benchmarks and debug runner).
- `checker_cache/` — saved benchmark input sets and expected performance outputs.
- `outputs/` — example program outputs.

## Build

To build the project (if a Makefile target is present):

```bash
make
```

This should produce the project's binaries (check console output or the `executables/` directory).

## Run

- Run the provided debug or simulation binary (if built):

```bash
# Example: run the debug binary if present
./executables/sequential.debug

# Or run a compiled simulation binary (name may vary)
./simulation
```

- To run the benchmark harness that exercises multiple inputs:

```bash
./run_bench.sh
```

- To generate input files for testing/benchmarks:

```bash
python3 gen.py
```

Adjust arguments to `gen.py` if required (see the script header or source for options).

## Inputs & Outputs

- Example inputs and cached results live in `checker_cache/`.
- Program outputs (example) are in `outputs/`.

## Notes

- This README is intentionally short. For details about program options and command-line flags, inspect the top of `simulation.cc` and `traffic.cc`, or open the `Makefile` to see available build targets.
- If you add or change binaries, update the `executables/` folder or the `Makefile` accordingly.

## Contact / Author

For questions about the assignment or code, contact the repository owner or refer to your course staff.

---

(Simple README created for the SCH assignment.)
