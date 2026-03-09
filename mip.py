from miscellaneous import read_instance
from gurobipy import Model, GRB, quicksum

def solve_mip(instance_number, number_of_runways, objective="penalty"):

    instance = read_instance(instance_number) 

    number_of_planes = instance['number_of_planes']
    #freeze_time = instance['freeze_time'] #The freeze time is relevant in the context of a dynamic version of the problem, but for the static version we can ignore it. It i the time after which we can no longer change the airplan landing time.
    #planes = instance['planes']

    #Sets and parameters

    P = range(number_of_planes)  # Set of planes
    if number_of_runways >= 2:
        R = range(number_of_runways)  # Set of runways
        s = [[0 for _ in range(number_of_planes)] for _ in range(number_of_planes)]

    E = instance['earliest_landing_vector']  # Earliest landing times
    T = instance['target_landing_vector']  # Target landing times
    L = instance['latest_landing_vector']  # Latest landing times

    S = instance['separation_times_matrix']  # Separation times between planes

    g = instance['penalty_early_vector']  # Penalty for early landing
    h = instance['penalty_late_vector']  # Penalty for late landing

    # U is the set of pairs (i,j) of planes for which we are uncertain whether plane i lands before plane j or not
    U = [(i, j) for i in P for j in P if i != j and (E[i] <= E[j] <= L[i] or E[i] <= L[j] <= L[i] or E[j] <= E[i] <= L[j] or E[j] <= L[i] <= L[j])]

    # V is he set of pairs (i,j) of planes for which i definitely lands before j (but for which the separation constraint is not automatically satisfied)
    V = [(i, j) for i in P for j in P if i != j and L[i] < E[j] and L[i] + S[i][j] > E[j]]

    # W is the set of pairs (i,j) of planes for which i definitely lands before j (and for which the separation constraint is automatically satisfied)
    W = [(i, j) for i in P for j in P if i != j and L[i] < E[j] and L[i] + S[i][j] <= E[j]]

    #Variables
    model = Model("Airplane_Landing_Problem")

    x = model.addVars(P, vtype=GRB.CONTINUOUS, name="x")  # Continuous variable for landing time of each plane
    alpha = model.addVars(P, vtype=GRB.CONTINUOUS, name="alpha")  # Continuous variable for how much earlier than target time
    beta = model.addVars(P, vtype=GRB.CONTINUOUS, lb=0,name="beta")  # Continuous variable for how much later than target time
    delta = model.addVars([(i, j) for i in P for j in P if i != j], vtype=GRB.BINARY, name="delta")  # Binary variable for ordering of planes
    if objective == "makespan":
        Z_max = model.addVar(vtype=GRB.CONTINUOUS, name="Z_max")  # Continuous variable for maximum penalty
    if number_of_runways >= 2:
        y = model.addVars(P, R, vtype=GRB.BINARY, name="y")  # Binary variable for runway assignment
        z = model.addVars([(i, j) for i in P for j in P if i != j], vtype=GRB.BINARY, name="z")  # Binary variable for separation on the same runway

    #Objective function
    if objective == "penalty":
        model.setObjective(quicksum(g[i] * alpha[i] + h[i] * beta[i] for i in P), GRB.MINIMIZE)
    elif objective == "total_time":
        model.setObjective(quicksum(x[i] for i in P), GRB.MINIMIZE)
    elif objective == "makespan":
        model.setObjective(Z_max, GRB.MINIMIZE)

    #Constraints
    for i in P:
        model.addConstr(x[i] >= E[i], name=f"Earliest_Landing_{i}")
        model.addConstr(x[i] <= L[i], name=f"Latest_Landing_{i}")
        model.addConstr(alpha[i] >= T[i] - x[i], name=f"Alpha_Definition_{i}")
        model.addConstr(beta[i] >= x[i] - T[i], name=f"Beta_Definition_{i}")
        model.addConstr(beta[i] <= L[i] - T[i], name=f"Beta_Limit_{i}")
        model.addConstr(x[i] == T[i] - alpha[i] + beta[i], name=f"Landing_Time_Definition_{i}")
        for j in P:
            if j > i:
                model.addConstr(delta[i, j] + delta[j, i] == 1, name=f"Ordering_{i}_{j}")

    for (i, j) in W + V:
        model.addConstr(delta[i,j] == 1, name=f"Ordering_{i}_{j}")

    if number_of_runways == 1:
        for (i, j) in V:
            model.addConstr(x[j] >= x[i] + S[i][j], name=f"Separation_{i}_{j}")

        for (i,j) in U:
            model.addConstr(x[j] >= x[i] + S[i][j]*delta[i,j] - (L[i]-E[j])*delta[j,i], name=f"Separation_Uncertain_{i}_{j}")

    if objective == "makespan":
        for i in P:
            model.addConstr(Z_max >= x[i])

    if number_of_runways >= 2:
        for i in P:
            model.addConstr(quicksum(y[i, r] for r in R) == 1, name=f"Runway_Assignment_{i}")

        for i in P:
            for j in P:
                if j>i:
                    model.addConstr(z[i,j] == z[j,i], name=f"Symmetry_z_{i}_{j}")
                    for r in R:
                        model.addConstr(z[i,j] >= y[i,r] + y[j,r] - 1, name=f"Separation_Same_Runway_{i}_{j}_Runway_{r}")

        for (i, j) in V:
            model.addConstr(x[j] >= x[i] + S[i][j]*z[i,j] + s[i][j]*(1-z[i,j]), name=f"Separation_Same_Runway_V_{i}_{j}")

        for (i, j) in U:
            model.addConstr(x[j] >= x[i] + S[i][j]*z[i,j] + s[i][j]*(1-z[i,j]) - (L[i]+max(S[i][j], s[i][j])-E[j])*delta[j,i], name=f"Separation_Same_Runway_U_{i}_{j}")

    model.optimize()

    if number_of_runways == 1:
        runway_assignment = [0 for _ in P]  # All planes assigned to runway 0
        landing_times = [x[i].X for i in P]
    else:
        runway_assignment = [r for i in P for r in R if y[i, r].X > 0.5]
        landing_times = [x[i].X for i in P]
    
    return landing_times, runway_assignment, model.ObjVal

