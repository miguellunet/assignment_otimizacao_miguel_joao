from miscellaneous import read_instance
from ortools.sat.python import cp_model
import time


def solve_cp(instance_number, number_of_runways, objective="penalty", time_limit=None):

    """
    Implementation note on the transition matrix S:
        CP-SAT's NoOverlap does NOT accept a sequence-dependent transition distance
        matrix (unlike IBM CP Optimizer). We therefore enforce the transition distances S_ij over
        ALL ordered pairs through reified constraints, which exactly matches the
        all-pairs semantics stated in the formulation
        ("x_j >= x_i + S_ij whenever i lands before j on the same runway").
        The NoOverlap calls declare the shared-resource structure; the distances
        themselves are imposed through the reified separation constraints below.
    """

    instance = read_instance(instance_number)

    number_of_planes = instance['number_of_planes']
    P = range(number_of_planes)

    # All time parameters are integer-valued in the benchmark instances
    E = [int(e) for e in instance['earliest_landing_vector']]
    T = [int(t) for t in instance['target_landing_vector']]
    L = [int(l) for l in instance['latest_landing_vector']]
    S = [[int(s) for s in row] for row in instance['separation_times_matrix']]

    g = instance['penalty_early_vector']   # float (original scale)
    h = instance['penalty_late_vector']    # float (original scale)

    # CP-SAT operates exclusively over integers. Penalty coefficients may have up to
    # 2 decimal places (e.g., 1.01, 1.99), so we scale by 100 to obtain integers.
    SCALE = 100
    g_scaled = [int(round(gi * SCALE)) for gi in g]
    h_scaled = [int(round(hi * SCALE)) for hi in h]

    if number_of_runways >= 2:
        R = range(number_of_runways)

    # ----- Sets U, V, W (same logic as MIP) -----
    U_set = set()
    for i in P:
        for j in P:
            if i != j and (E[i] <= E[j] <= L[i] or E[i] <= L[j] <= L[i]
                           or E[j] <= E[i] <= L[j] or E[j] <= L[i] <= L[j]):
                U_set.add((i, j))

    V = [(i, j) for i in P for j in P
         if i != j and L[i] < E[j] and L[i] + S[i][j] > E[j]]

    W = [(i, j) for i in P for j in P
         if i != j and L[i] < E[j] and L[i] + S[i][j] <= E[j]]

    U = list(U_set)

    # ================================================================
    # Model
    # ================================================================
    start_time = time.time()
    model = cp_model.CpModel()

    # ----- Variables and Domains -----

    # Landing time of each aircraft = start of its interval variable
    x = {i: model.new_int_var(E[i], L[i], f'x_{i}') for i in P}

    # Earliness and lateness relative to target
    alpha = {i: model.new_int_var(0, max(T[i] - E[i], 0), f'alpha_{i}') for i in P}
    beta  = {i: model.new_int_var(0, max(L[i] - T[i], 0), f'beta_{i}')  for i in P}

    # Earliness / lateness definition via max (replaces 5 MIP constraints per aircraft)
    for i in P:
        model.add_max_equality(alpha[i], [T[i] - x[i], model.new_constant(0)])
        model.add_max_equality(beta[i],  [x[i] - T[i], model.new_constant(0)])

    # Interval variable for each aircraft: intv_i = Interval(start=x_i, size=0)
    intv = {i: model.new_fixed_size_interval_var(x[i], 0, f'intv_{i}') for i in P}

    # Ordering BoolVars - only for pairs in U whose order is uncertain.
    # One BoolVar per unordered pair {i,j} with i < j:
    #   before[i,j] = True  <=>  aircraft i lands before aircraft j
    before = {}
    for i in P:
        for j in range(i + 1, number_of_planes):
            if (i, j) in U_set or (j, i) in U_set:
                before[i, j] = model.new_bool_var(f'before_{i}_{j}')

    def i_before_j(i, j):
        """Return the literal expressing 'aircraft i lands before aircraft j'."""
        if i < j:
            return before[i, j]
        else:
            return before[j, i].negated()

    # ----- Constraints -----

    if number_of_runways == 1:
        # ============================================================
        # Single Runway
        # ============================================================

        # NoOverlap on the single shared runway (Eq. cp:no_overlap_single).
        model.add_no_overlap([intv[i] for i in P])

        # Transition distances S_ij, enforced over ALL pairs (all-pairs semantics).
        # V pairs: order known in advance -> direct separation (Eq. cp:precedence_V).
        for (i, j) in V:
            model.add(x[j] >= x[i] + S[i][j])

        # U pairs: order uncertain -> reified disjunctive separation.
        # Iterating over all ordered pairs (i,j) in U covers both directions.
        for (i, j) in U:
            lit = i_before_j(i, j)
            model.add(x[j] >= x[i] + S[i][j]).only_enforce_if(lit)

    else:
        # ============================================================
        # Multiple Runways
        # ============================================================

        # Optional interval intv_ir for each (aircraft, runway) pair, all sharing
        # the same start x_i. Presence literal lit[i,r] = aircraft i is on runway r.
        lit = {}
        opt_intv = {}
        for i in P:
            for r in R:
                lit[i, r] = model.new_bool_var(f'lit_{i}_{r}')
                opt_intv[i, r] = model.new_optional_fixed_size_interval_var(
                    x[i], 0, lit[i, r], f'intv_{i}_{r}'
                )
            # ALTERNATIVE constraint (Eq. cp:alternative): exactly one optional
            # interval present. Since all share start x_i, the present one fixes the
            # landing time, matching Alternative(intv_i, {intv_ir : r}).
            model.add_exactly_one([lit[i, r] for r in R])

        # NoOverlap per runway over the optional intervals (Eq. cp:no_overlap_multi).
        # Absent intervals are automatically ignored by the solver.
        for r in R:
            model.add_no_overlap([opt_intv[i, r] for i in P])

        # Symmetry breaking: fix first aircraft to runway 0.
        # Eliminates the |R| equivalent permutations of runway labels for aircraft 0.
        model.add(lit[0, 0] == 1)

        # Transition distances S_ij, enforced over ALL pairs and conditioned on the
        # two aircraft sharing a runway (s_ij = 0 => no cross-runway separation).
        # V pairs: order known in advance.
        for (i, j) in V:
            for r in R:
                model.add(x[j] >= x[i] + S[i][j]).only_enforce_if([lit[i, r], lit[j, r]])

        # U pairs: order uncertain.
        for (i, j) in U:
            ordering_lit = i_before_j(i, j)
            for r in R:
                model.add(
                    x[j] >= x[i] + S[i][j]
                ).only_enforce_if([ordering_lit, lit[i, r], lit[j, r]])

    # ----- Makespan (if needed) -----
    if objective == "makespan":
        Z_max = model.new_int_var(min(E), max(L), 'Z_max')
        model.add_max_equality(Z_max, [x[i] for i in P])

    # ----- Objective Function -----
    if objective == "penalty":
        model.minimize(
            sum(g_scaled[i] * alpha[i] + h_scaled[i] * beta[i] for i in P)
        )
    elif objective == "total_time":
        model.minimize(sum(x[i] for i in P))
    elif objective == "makespan":
        model.minimize(Z_max)

    # ================================================================
    # Solve
    # ================================================================
    solver = cp_model.CpSolver()
    if time_limit is not None:
        solver.parameters.max_time_in_seconds = time_limit
    solver.parameters.num_workers = 8

    solver.parameters.absolute_gap_limit = 0.0
    solver.parameters.relative_gap_limit = 0.0
    status = solver.Solve(model)

    end_time = time.time()
    time_taken = end_time - start_time

    # ================================================================
    # Extract results
    # ================================================================
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        landing_times = [float(solver.value(x[i])) for i in P]

        if number_of_runways == 1:
            runway_assignment = [0 for _ in P]
        else:
            runway_assignment = []
            for i in P:
                for r in R:
                    if solver.value(lit[i, r]):
                        runway_assignment.append(r)
                        break

        # Compute all three objectives from the solution (original scale)
        penalty_objective = sum(
            g[i] * max(T[i] - landing_times[i], 0) +
            h[i] * max(landing_times[i] - T[i], 0)
            for i in P
        )
        total_time_objective = sum(landing_times)
        makespan_objective = max(landing_times)
        time_diff_to_target = [landing_times[i] - T[i] for i in P]

        # Upper and lower bounds (unscale penalty objective)
        if objective == "penalty":
            ub = solver.objective_value / SCALE
            lb = solver.best_objective_bound / SCALE
        else:
            ub = solver.objective_value
            lb = solver.best_objective_bound

        # Relative optimality gap (consistent with Gurobi's MIPGap)
        if abs(ub) > 1e-10:
            gap = abs(ub - lb) / abs(ub)
        elif abs(lb) < 1e-10:
            gap = 0.0
        else:
            gap = float('inf')

    else:
        landing_times = None
        time_diff_to_target = None
        runway_assignment = None
        penalty_objective = None
        total_time_objective = None
        makespan_objective = None
        ub = None
        lb = None
        gap = None

    return (landing_times, time_diff_to_target, runway_assignment,
            penalty_objective, total_time_objective, makespan_objective,
            time_taken, ub, lb, gap)
