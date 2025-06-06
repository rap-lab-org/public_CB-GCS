# Reproduce Experiments

- Prerequisite:
    - We use Gurobi for GCS and MILP, a [license](https://www.gurobi.com/academia/academic-program-and-licenses/) is required.
    - Install all dependencies by python package manager (e.g., `uv`): `uv sync`

- Generate scenario files: `python ./cbs/gen.py`
    - Generate scenarios for the experiment
    - All generated scenarios are store in folder `gcs-expr-scen`

- Run experiment: `python ./cbs/expr.py -dt 0.2 -t 10 -tlimit 100`
