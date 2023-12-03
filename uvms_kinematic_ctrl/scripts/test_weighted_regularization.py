import numpy as np

sigma0 = 0.005
v_smooth = 0.005


def bell_function(sigma, weight_factor, sigma_max):
    if sigma / sigma_max < weight_factor * sigma0:
        return weight_factor * v_smooth * sigma_max * (1 - sigma / (weight_factor * sigma_max * sigma0)) ** 2
    else:
        return 0


def continuous_pseudo_inverse(X: np.ndarray, A: np.ndarray, Q: np.ndarray, W_sqrt_inv: np.ndarray, P0_sqrt: np.ndarray,
                              sigma_max, weight_reg=False) -> np.ndarray:
    n_states = np.shape(X)[1]
    identity = np.eye(n_states)
    eta = 1.0
    if weight_reg:
        tmp = X.transpose() @ P0_sqrt @ A @ P0_sqrt @ X + eta * (identity - Q).transpose() @ P0_sqrt @ P0_sqrt @ (
                    identity - Q)
    else:
        tmp = X.transpose() @ P0_sqrt @ A @ P0_sqrt @ X + eta * (identity - Q).transpose() @ (identity - Q)
    U_1, sigmas_1, V_t_1 = np.linalg.svd(tmp)
    sigma_max[0] = min(max(sigma_max[0], sigmas_1[0]), 1)
    tmp = W_sqrt_inv @ tmp @ W_sqrt_inv
    U, sigmas, V_t = np.linalg.svd(tmp)
    P = np.zeros((n_states, n_states))
    for i in range(n_states):
        if (sigmas[i] < 1e-12):
            sigmas[i] = 0.0
        P[i, i] = bell_function(sigmas[i], sigmas[i] / max(sigmas_1[i], 1e-10),
                                sigma_max[0])  # sigmas[i] / max(sigmas_1[i], 1e-10), sigma_max[0])# 1.0, 1.0)#
    return np.linalg.pinv(tmp + V_t.transpose() @ P @ V_t) @ W_sqrt_inv @ X.transpose() @ P0_sqrt @ A @ A @ P0_sqrt


def get_velocity(x: np.ndarray, velocity: np.ndarray, J: np.ndarray, projector: np.ndarray, A: np.ndarray,
                 W_sqrt_inv: np.ndarray, P0_sqrt: np.ndarray, sigma_max, weight_reg=False) -> np.ndarray:
    n_states = np.shape(J)[1]
    identity = np.eye(n_states)
    W = J @ projector @ W_sqrt_inv @ continuous_pseudo_inverse(J @ projector, A, projector, W_sqrt_inv, P0_sqrt,
                                                               sigma_max, weight_reg=weight_reg)
    return projector @ W_sqrt_inv @ continuous_pseudo_inverse(J @ projector, A, identity, W_sqrt_inv, P0_sqrt,
                                                              sigma_max, weight_reg=weight_reg) @ W @ (x - J @ velocity)


def update_projector(J: np.ndarray, projector: np.ndarray, A: np.ndarray, W_sqrt_inv: np.ndarray, P0_sqrt: np.ndarray,
                     sigma_max, weight_reg=False) -> np.ndarray:
    n_states = np.shape(J)[1]
    identity = np.eye(n_states)
    projector = projector @ (
                identity - W_sqrt_inv @ continuous_pseudo_inverse(J @ projector, A, identity, W_sqrt_inv, P0_sqrt,
                                                                  sigma_max, weight_reg=weight_reg) @ J @ projector)
    return projector


def main():
    final_velocity_minimization = True

    # jacobian:
    x_ref_list = [np.array([[0.1], [0.1]])]
    n_states = 3
    if n_states == 2:
        J_list = [np.array([[-1, -0.5],
                            [1, 1]])]

    elif n_states == 3:
        J_list = [np.array([[-1, -0.5, 1],
                            [1, 1, 1]])]

    else:
        print("Number of states not handled, return")
        return
    n_task = 2
    W_sqrt_inv = np.eye(n_states)

    N_weight = 5

    N = 1000

    sigma_max = [1e-8]

    dx_p = np.zeros((N_weight, N, n_task))
    dq_p = np.zeros((N_weight, N, n_states))
    a22 = np.linspace(0.0, 1.0, N)
    p22 = np.logspace(-2, 2, N_weight)
    w22 = np.logspace(-2, 2, N_weight)
    A = np.eye(n_task)

    # vary only joint space weighting
    dx_w = np.zeros((N_weight, N, n_task))
    dq_w = np.zeros((N_weight, N, n_states))
    W_sqrt_inv = np.eye(n_states)
    P0_sqrt = np.eye(n_states)
    for j in range(N_weight):
        W_sqrt_inv[1, 1] = 1 / np.sqrt(w22[j])
        for i in range(N):
            velocity = np.zeros((n_states, 1))
            projector = np.eye(n_states)
            A[1, 1] = a22[i]
            for x, J in zip(x_ref_list, J_list):
                velocity += get_velocity(x, velocity, J, projector, A, W_sqrt_inv, np.eye(n_task), sigma_max)
                projector = update_projector(J, projector, A, W_sqrt_inv, np.eye(n_task), sigma_max)
            # final minimization of velocity
            # U, s, V_t = np.linalg.svd(projector)
            # print(s)
            if final_velocity_minimization:
                velocity += get_velocity(np.zeros((n_states, 1)), velocity, np.eye(n_states), projector,
                                         np.eye(n_states), W_sqrt_inv, P0_sqrt, sigma_max)
            dq_w[j, i, :] = velocity.reshape(-1)
            dx_w[j, i, :] = (J_list[0] @ dq_w[j, i, :]).reshape(-1)
    print(sigma_max)
    # vary only task weighting
    W_sqrt_inv = np.eye(n_states)
    P0_sqrt = np.eye(n_states)
    for j in range(N_weight):
        P0_sqrt[1, 1] = np.sqrt(p22[j])
        for i in range(N):
            velocity = np.zeros((n_states, 1))
            projector = np.eye(n_states)
            A[1, 1] = a22[i]
            for x, J in zip(x_ref_list, J_list):
                velocity += get_velocity(x, velocity, J, projector, A, W_sqrt_inv, np.eye(n_task), sigma_max)
                projector = update_projector(J, projector, A, W_sqrt_inv, np.eye(n_task), sigma_max)
            # final minimization of velocity
            if final_velocity_minimization:
                velocity += get_velocity(np.zeros((n_states, 1)), velocity, np.eye(n_states), projector,
                                         np.eye(n_states), W_sqrt_inv, P0_sqrt, sigma_max)
            dq_p[j, i, :] = velocity.reshape(-1)
            dx_p[j, i, :] = (J_list[0] @ dq_p[j, i, :]).reshape(-1)

    # vary both weightings
    dx_wp = np.zeros((N_weight, N, n_task))
    dq_wp = np.zeros((N_weight, N, n_states))
    W_sqrt_inv = np.eye(n_states)
    P0_sqrt = np.eye(n_states)
    for j in range(N_weight):
        P0_sqrt[1, 1] = np.sqrt(p22[j])
        W_sqrt_inv[1, 1] = 1 / np.sqrt(w22[j])
        for i in range(N):
            velocity = np.zeros((n_states, 1))
            projector = np.eye(n_states)
            A[1, 1] = a22[i]
            for x, J in zip(x_ref_list, J_list):
                velocity += get_velocity(x, velocity, J, projector, A, W_sqrt_inv, np.eye(n_task), sigma_max)
                projector = update_projector(J, projector, A, W_sqrt_inv, np.eye(n_task), sigma_max)
            # final minimization of velocity
            if final_velocity_minimization:
                velocity += get_velocity(np.zeros((n_states, 1)), velocity, np.eye(n_states), projector,
                                         np.eye(n_states), W_sqrt_inv, P0_sqrt, sigma_max)
            dq_wp[j, i, :] = velocity.reshape(-1)
            dx_wp[j, i, :] = (J_list[0] @ dq_wp[j, i, :]).reshape(-1)

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(n_task, 1, sharex='all')
    fig.suptitle("Task velocities for variation of joint velocity weighting")
    for j in range(N_weight):
        for i in range(n_task):
            axes[i].plot(a22, dx_w[j, :, i], label="w22: " + str(p22[j]))
    axes[-1].set_xlabel("activation value a(2,2)")
    for i in range(n_task):
        maximum = np.max(dx_w[:, :, i])
        minimum = np.min(dx_w[:, :, i])
        min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
        max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
        axes[i].set_ylim(min_lim, max_lim)
        axes[i].set_ylabel("task velocity dx" + str(i))
        axes[i].legend()
        axes[i].grid()

    fig, axes = plt.subplots(n_states, 1, sharex='all')
    fig.suptitle("Joint velocities for variation of joint velocity weighting")
    for j in range(N_weight):
        for i in range(n_states):
            axes[i].plot(a22, dq_w[j, :, i], label="w22: " + str(p22[j]))
    axes[-1].set_xlabel("activation value a(2,2)")
    for i in range(n_states):
        maximum = np.max(dq_w[:, :, i])
        minimum = np.min(dq_w[:, :, i])
        min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
        max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
        axes[i].set_ylim(min_lim, max_lim)
        axes[i].set_ylabel("joint velocity dq" + str(i))
        axes[i].legend()
        axes[i].grid()

    fig, axes = plt.subplots(n_task, 1, sharex='all')
    fig.suptitle("Task velocities for variation of task velocity weighting")
    for j in range(N_weight):
        for i in range(n_task):
            axes[i].plot(a22, dx_p[j, :, i], label="p22: " + str(p22[j]))
    axes[-1].set_xlabel("activation value a(2,2)")
    for i in range(n_task):
        maximum = np.max(dx_p[:, :, i])
        minimum = np.min(dx_p[:, :, i])
        min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
        max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
        axes[i].set_ylim(min_lim, max_lim)
        axes[i].set_ylabel("task velocity dx" + str(i))
        axes[i].legend()
        axes[i].grid()

    fig, axes = plt.subplots(n_states, 1, sharex='all')
    fig.suptitle("Joint velocities for variation of task velocity weighting")
    for j in range(N_weight):
        for i in range(n_states):
            axes[i].plot(a22, dq_p[j, :, i], label="p22: " + str(p22[j]))
    axes[-1].set_xlabel("activation value a(2,2)")
    for i in range(n_states):
        maximum = np.max(dq_p[:, :, i])
        minimum = np.min(dq_p[:, :, i])
        min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
        max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
        axes[i].set_ylim(min_lim, max_lim)
        axes[i].set_ylabel("joint velocity dq" + str(i))
        axes[i].legend()
        axes[i].grid()

    fig, axes = plt.subplots(n_task, 1, sharex='all')
    fig.suptitle("Task velocities for variation both weightings")
    for j in range(N_weight):
        for i in range(n_task):
            axes[i].plot(a22, dx_wp[j, :, i], label="w22, p22: " + str(p22[j]))
    axes[-1].set_xlabel("activation value a(2,2)")
    for i in range(n_task):
        maximum = np.max(dx_wp[:, :, i])
        minimum = np.min(dx_wp[:, :, i])
        min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
        max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
        axes[i].set_ylim(min_lim, max_lim)
        axes[i].set_ylabel("task velocity dx" + str(i))
        axes[i].legend()
        axes[i].grid()

    fig, axes = plt.subplots(n_states, 1, sharex='all')
    fig.suptitle("Joint velocities for variation of both weightings")
    for j in range(N_weight):
        for i in range(n_states):
            axes[i].plot(a22, dq_wp[j, :, i], label="w22, p22: " + str(p22[j]))
    axes[-1].set_xlabel("activation value a(2,2)")
    for i in range(n_states):
        maximum = np.max(dq_wp[:, :, i])
        minimum = np.min(dq_wp[:, :, i])
        min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
        max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
        axes[i].set_ylim(min_lim, max_lim)
        axes[i].set_ylabel("joint velocity dq" + str(i))
        axes[i].legend()
        axes[i].grid()

    plt.show()


if __name__ == "__main__":
    main()
