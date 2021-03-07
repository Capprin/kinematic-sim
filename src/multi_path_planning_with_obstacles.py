from mip import Model, xsum, GRB, MINIMIZE, BINARY
from mip import INTEGER, CONTINUOUS, OptimizationStatus
import matplotlib.pyplot as plt
from time import time


def solve_multi_with_obstacles(world_dim=(100, 100),
                               agents=3,
                               finite_time_horizon=20, max_vel=10,
                               max_accel=1, initial_states=[[]],
                               final_states=[[]],
                               obstacle=[((), ())]):

    m = Model("little_pp_solver", sense=MINIMIZE, solver_name=GRB)

    # weights or some shit
    q, r, p = 1, 1, 1
    sample_time = 0.25
    N = int(finite_time_horizon / sample_time)

    M = 1e4

    d_x = 5
    d_y = 5

    state_dim = 4
    input_dim = 2

    state_bounds = [(0, world_dim[0]-1),
                    (0, world_dim[1]-1),
                    (-1*max_vel, max_vel),
                    (-1*max_vel, max_vel)]

    # VARIABLES TO OPTIMIZE ##
    # actual state & input
    S = [[[m.add_var(var_type=CONTINUOUS, lb=state_bounds[j][0], ub=state_bounds[j][1])
           for j in range(state_dim)] for i in range(N)] for p in range(agents)]
    U = [[[m.add_var(var_type=CONTINUOUS,
                     lb=-1*max_accel,
                     ub=max_accel)
           for k in range(input_dim)] for i in range(N)] for p in range(agents)]
    # slack state & input
    W = [[[m.add_var(var_type=CONTINUOUS)
           for j in range(state_dim)] for i in range(N)] for p in range(agents)]
    V = [[[m.add_var(var_type=CONTINUOUS)
           for k in range(input_dim)] for i in range(N)] for p in range(agents)]

    T = [[[[m.add_var(var_type=BINARY) for k in range(4)]
           for i in range(N)]
          for c in range(len(obstacles))]
         for p in range(agents)]

    B = [[[[m.add_var(var_type=BINARY) for k in range(4)]
           for i in range(N)]
          for q in range(agents)]
         for p in range(agents)]

    print(len(B[0]))

    # OBJECTIVE FUNCTION ##
    m.objective = 0
    for p in range(agents):
        for j in range(state_dim):
            m.objective = m.objective + xsum(q*W[p][i][j] for i in range(1, N))
        for k in range(input_dim):
            m.objective = m.objective + xsum(r*V[p][i][k] for i in range(N))
    # m.objective = m.objective + terminal_cost

    # CONSTRAINTS ##
    for p in range(agents):  # for all agents...
        for i in range(N):       # for all timesteps...
            for j in range(state_dim):
                m += S[p][i][j] - final_states[p][j] <= W[p][i][j]
                m += -1*S[p][i][j] + final_states[p][j] <= W[p][i][j]
            for k in range(input_dim):
                m += U[p][i][k] <= V[p][i][k]
                m += -1*U[p][i][k] <= V[p][i][k]
            # Smoothbrain state transition constraints
            if i < N-1:
                # next position
                m += S[p][i+1][2] == S[p][i][2] + U[p][i][0]
                m += S[p][i+1][3] == S[p][i][3] + U[p][i][1]
                m += S[p][i+1][0] == S[p][i][0] + S[p][i+1][2]*sample_time
                m += S[p][i+1][1] == S[p][i][1] + S[p][i+1][3]*sample_time

    # TODO: maintain a minimum distance away from obstacles
    for p in range(agents):
        for c in range(len(obstacles)):

            x_min, y_min = obstacles[c][0]
            x_max, y_max = obstacles[c][1]

            for i in range(N):
                m += S[p][i][0] <= x_min + M*T[p][c][i][0]
                m += -1*S[p][i][0] <= -1*x_max + M*T[p][c][i][1]
                m += S[p][i][1] <= y_min + M*T[p][c][i][2]
                m += -1*S[p][i][1] <= -1*y_max + M*T[p][c][i][3]

                m += xsum(T[p][c][i][k] for k in range(4)) <= 3

    for p in range(agents):
        for q in range(p+1, agents):
            for i in range(N):
                m += S[p][i][0] - S[q][i][0] >= d_x - M*B[p][q][i][0]
                m += S[q][i][0] - S[p][i][0] >= d_x - M*B[p][q][i][1]
                m += S[p][i][1] - S[q][i][1] >= d_y - M*B[p][q][i][2]
                m += S[q][i][1] - S[p][i][1] >= d_y - M*B[p][q][i][3]

                m += xsum(B[p][q][i][k] for k in range(4)) <= 3

    # initial conditions:
    for p in range(agents):
        for j in range(state_dim):
            m += S[p][0][j] == initial_states[p][j]

    m.optimize()

    status = m.optimize(max_seconds=300)
    if status == OptimizationStatus.OPTIMAL:
        print('optimal solution cost {} found'.format(m.objective_value))
    elif status == OptimizationStatus.FEASIBLE:
        print('sol.cost {} found, best possible: {}'.format(m.objective_value,
                                                            m.objective_bound))
    elif status == OptimizationStatus.NO_SOLUTION_FOUND:
        print('no feasible solution found, lower bound is: {}'
              .format(m.objective_bound))

    states = []

    if status == OptimizationStatus.OPTIMAL or\
       status == OptimizationStatus.FEASIBLE:

        states = []
        for p in range(agents):
            states.append([])
            for i in range(N):
                states[p].append([tmp.x for tmp in S[p][i]])

    return states


def draw_path(states, world_dim, obstacles):

    for s in states:
        x = []
        y = []

        for state in s:

            x.append(state[0])
            y.append(state[1])

        plt.scatter(x[0], y[0], color="green", marker='o', zorder=100)
        plt.scatter(x[-1], y[-1], color="red", marker='x', zorder=200)
        plt.plot(x, y, 'bo--', zorder=0)
        plt.legend(["path", "start", "goal"])

    for obstacle in obstacles:
        min_x, min_y = obstacle[0]
        max_x, max_y = obstacle[1]

        bl = obstacle[0]
        tr = obstacle[1]
        br = (max_x, min_y)
        tl = (min_x, max_y)

        x = []
        y = []

        for a, b in [bl, tl, tr, br, bl]:
            x.append(a)
            y.append(b)

        plt.plot(x, y, color="red")

    plt.xlim([0, world_dim[0]])
    plt.ylim([0, world_dim[0]])
    plt.grid()
    plt.show()


if __name__ == "__main__":

    world_dim = (50, 50)
    finite_time_horizon = 50
    max_vel = 10
    max_accel = 1
    agents = 3

    initial_states = [[5, 21, 0, 0],
                      [5, 10, 0, 0],
                      [2, 15, 0, 0]]

    final_states = [[40, 45, 0, 0],
                    [50, 50, 0, 0],
                    [35, 45, 0, 0]]

    obstacles = [((-1, 30), (10, 35)),
                 ((15, 20), (30, 49)),
                 ((15, -1), (30, 18))]

    t = time()
    states = solve_multi_with_obstacles(world_dim, agents, finite_time_horizon,
                                        max_vel, max_accel, initial_states,
                                        final_states, obstacles)
    print(time()-t)

    draw_path(states, world_dim=world_dim, obstacles=obstacles)
