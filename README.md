# Aircraft Landing Problem — MIP vs CP

Comparison of Mixed-Integer Programming (MIP) and Constraint Programming (CP) approaches for the Aircraft Landing Problem, developed for the Optimization course in the Doctoral Program in Industrial Engineering and Management at the University of Porto.

## Problem

Given a set of aircraft with time windows and target landing times, determine when and on which runway each aircraft should land, while respecting minimum separation times between consecutive landings on the same runway. Three objective functions are considered: minimization of weighted earliness/lateness penalties, minimization of total landing time, and minimization of makespan.

The benchmark instances are taken from [Beasley et al. (2000)](https://doi.org/10.1016/S0377-2217(99)00369-3), ranging from 10 to 500 aircraft.

## Repository Structure

```
├── mip.py                  # MIP formulation (Gurobi)
├── cp.py                   # CP formulation (OR-Tools CP-SAT)
├── cp.py                   # First CP formulation (not taking advantage of global constraints)
├── miscellaneous.py        # Instance parser
├── instances/              # Benchmark instances (airland1.txt – airland13.txt)
├── export_results.ipynb    # Runs full factorial experiment for both solvers
├── export_plots.ipynb      # Generates tables, plots, and MIP vs CP comparison
├── results/
│   ├── optimization_results_all.csv   # All results (both solvers)
│   ├── results_plots/                 # Normalized objective plots, trade-off plots
│   └── results_solutions/             # Landing schedule and feasibility check plots
```

## Requirements

```
pip install gurobipy ortools pandas matplotlib tabulate
```

- **Gurobi** requires a license (free academic licenses available at [gurobi.com](https://www.gurobi.com/academia/academic-program-and-licenses/)).
- **OR-Tools** is open-source and requires no license.

## Usage

Both solvers expose the same interface:

```python
from mip import solve_mip
from cp import solve_cp

# solve_mip / solve_cp(instance_number, number_of_runways, objective, time_limit)
#   instance_number: 1–13
#   number_of_runways: 1, 2, or 3
#   objective: "penalty", "total_time", or "makespan"
#   time_limit: seconds (optional)

landing_times, time_diff, runways, penalty, total_time, makespan, time, ub, lb, gap = \
    solve_cp(1, 2, "penalty", time_limit=300)
```

To run the full factorial experiment (13 instances × 3 objectives × 3 runway configs × 2 solvers = 234 runs):

1. Open `export_results.ipynb` and run all cells. Results are saved to `results/optimization_results_all.csv`.
2. Open `export_plots.ipynb` and run all cells to generate all tables, figures, and the MIP vs CP comparison.

## Experimental Design

| Parameter           | Values                                    |
|---------------------|-------------------------------------------|
| Instance            | 1–13 (10 to 500 aircraft)                 |
| Objective           | Penalty, Total landing time, Makespan     |
| Number of runways.  | 1, 2, 3                                   |
| Solver              | MIP (Gurobi), CP (CP-SAT)                 |
| Time limit per run  | 300 seconds                               |

## Authors

- João Marcelino — University of Porto
- Miguel Lunet — University of Porto
