"""pytest configuration: one BLAS thread per process (small arrays; see scripts/m2/_common.py)."""
import os

for _var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")
