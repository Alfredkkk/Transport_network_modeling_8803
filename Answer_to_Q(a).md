# Answer to Q(a): Multiclass UE/SO Extension Summary

This is the summary of extending the original single-class UE STA code to handle multiclass HDV/CV assignments and the corresponding SO formulation.

## Data/Parameters
- Wrapped the existing `Network` to build a `MultiClassSTA` module that stores per-class flows without changing the TNTP parser.
- Split every OD demand into HDV/CV shares (`configure_demand_split`) to support Scenario 1 (100% HDV) and Scenarios 2–3 (50/50).
- Derived CV-only capacities from the Levin & Boyles (2016) formula using the provided reaction times/vehicle length.
- Exposed scenario knobs for CV information effectiveness (`configure_eta`) and RSU link lists (`configure_rsu_links`).

## Cost Functions
- Implemented the HDV cost $c_{a,H} = l_a/v_a *[1 + (x_{a,H}/Q_{a,H} + x_{a,C}/Q_{a,C})^4]$ and CV cost $c_{a,C} = c_{a,H} * (1 - \eta_c * max(x_{a,C}/Q_{a,C}, \delta_{a,RSU}))$ as required by the CV scenario.
- Added marginal-cost derivatives per class (needed for SO Frank–Wolfe).

## Solvers
- Reused the single-class label-correcting shortest path, also reuse the all-or-nothing loader but looped over classes to keep HDV/CV flows separate.
- Generalized the Frank–Wolfe step to take vector flows (one per class) and compute the derivative with either travel times (UE) or marginal costs (SO).
- Added a multiclass `solve_system_optimal` routine that mirrors the UE loop but uses marginal costs for both direction finding and convergence tests.

## Outputs/Analysis
- Recorded iteration histories, total system travel time, and average travel time for each run so the notebooks (`Analysis.ipynb`) can compare UE vs SO results across scenarios and sensitivity studies.
- Added notebook utilities for parameter sweeps (CV penetration, RSU count, $\eta_c$) and spatial quadrant experiments using the multiclass solver.
