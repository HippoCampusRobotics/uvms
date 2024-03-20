import numpy as np

mumblue = (0, 100, 222)
mumred = (220, 33, 77)
mumgreen = (0, 140, 0)
mumpurple = (102, 0, 102)
mumorange = (255, 102, 0)
mumteal = (55, 200, 171)

c_mum = [
    tuple([float(x) / float(255) for x in y])
    for y in [mumblue, mumred, mumgreen, mumpurple, mumorange, mumteal]
]
c_mum_dict = {}
c_mum_dict['mumblue'] = c_mum[0]
c_mum_dict['mumred'] = c_mum[1]
c_mum_dict['mumgreen'] = c_mum[2]
c_mum_dict['mumpurple'] = c_mum[3]
c_mum_dict['mumorange'] = c_mum[4]
c_mum_dict['mumteal'] = c_mum[5]
sigma0 = 0.01
v_smooth = 0.01


def bell_function(sigma, weight_factor, sigma_max):
    if sigma / sigma_max < weight_factor * sigma0:
        return weight_factor * v_smooth * sigma_max * (
            1 - sigma / (weight_factor * sigma_max * sigma0))**2
    else:
        return 0


def continuous_pseudo_inverse(X: np.ndarray,
                              A: np.ndarray,
                              Q: np.ndarray,
                              W_sqrt_inv: np.ndarray,
                              P0_sqrt: np.ndarray,
                              sigma_max,
                              weight_reg=False) -> np.ndarray:
    n_states = np.shape(X)[1]
    identity = np.eye(n_states)
    eta = 1.0
    if weight_reg:
        tmp = X.transpose() @ P0_sqrt @ A @ P0_sqrt @ X + eta * (
            identity - Q).transpose() @ P0_sqrt @ P0_sqrt @ (identity - Q)
    else:
        tmp = X.transpose() @ P0_sqrt @ A @ P0_sqrt @ X + eta * (
            identity - Q).transpose() @ (identity - Q)
    U_1, sigmas_1, V_t_1 = np.linalg.svd(tmp)
    sigma_max[0] = min(max(sigma_max[0], sigmas_1[0]), 1)
    tmp = W_sqrt_inv @ tmp @ W_sqrt_inv
    U, sigmas, V_t = np.linalg.svd(tmp)
    P = np.zeros((n_states, n_states))
    for i in range(n_states):
        if (sigmas[i] < 1e-12):
            sigmas[i] = 0.0
        P[i, i] = bell_function(
            sigmas[i], sigmas[i] / max(sigmas_1[i], 1e-10), sigma_max[0]
        )  # sigmas[i] / max(sigmas_1[i], 1e-10), sigma_max[0])# 1.0, 1.0)#
    return np.linalg.pinv(tmp +
                          V_t.transpose() @ P @ V_t) @ W_sqrt_inv @ X.transpose(
                          ) @ P0_sqrt @ A @ A @ P0_sqrt


def get_velocity(x: np.ndarray,
                 velocity: np.ndarray,
                 J: np.ndarray,
                 projector: np.ndarray,
                 A: np.ndarray,
                 W_sqrt_inv: np.ndarray,
                 P0_sqrt: np.ndarray,
                 sigma_max,
                 weight_reg=False) -> np.ndarray:
    n_states = np.shape(J)[1]
    identity = np.eye(n_states)
    W = J @ projector @ W_sqrt_inv @ continuous_pseudo_inverse(
        J @ projector,
        A,
        projector,
        W_sqrt_inv,
        P0_sqrt,
        sigma_max,
        weight_reg=weight_reg)
    return projector @ W_sqrt_inv @ continuous_pseudo_inverse(
        J @ projector,
        A,
        identity,
        W_sqrt_inv,
        P0_sqrt,
        sigma_max,
        weight_reg=weight_reg) @ W @ (x - J @ velocity)


def update_projector(J: np.ndarray,
                     projector: np.ndarray,
                     A: np.ndarray,
                     W_sqrt_inv: np.ndarray,
                     P0_sqrt: np.ndarray,
                     sigma_max,
                     weight_reg=False) -> np.ndarray:
    n_states = np.shape(J)[1]
    identity = np.eye(n_states)
    projector = projector @ (identity - W_sqrt_inv @ continuous_pseudo_inverse(
        J @ projector,
        A,
        identity,
        W_sqrt_inv,
        P0_sqrt,
        sigma_max,
        weight_reg=weight_reg) @ J @ projector)
    return projector


def main():  # noqa: C901
    idx_activation = 1
    idx_weighting = 1
    # jacobian:
    x_ref_list = [np.array([[0.1], [0.1]])]
    n_states = 3
    if n_states == 2:
        J_list = [np.array([[-1, -0.5], [1, 1]])]

    elif n_states == 3:
        J_list = [np.array([[-1, -0.5, 1], [1, 1, -0.5]])]

    else:
        print("Number of states not handled, return")
        return
    n_task = 2
    """
    #jacobian:
    x_ref_list = [np.array([[0.1]])]
    n_states = 2
    J_list = [np.array([[-1, -0.5]])]
    n_task = 1
    """

    W_sqrt_inv = np.eye(n_states)

    N_weight = 3

    N = 500

    sigma_max = [1e-8]

    dx_p = np.zeros((N_weight, N, n_task))
    dq_p = np.zeros((N_weight, N, n_states))
    a22 = np.linspace(0.0, 1.0, N)
    p22 = np.logspace(-1, 1, N_weight)
    w22 = np.logspace(-1, 1, N_weight)
    A = np.eye(n_task)

    # vary only joint space weighting, no final velocity minimization:
    final_velocity_minimization = False
    dx_w = np.zeros((N_weight, N, n_task))
    dq_w = np.zeros((N_weight, N, n_states))
    W_sqrt_inv = np.eye(n_states)
    P0_sqrt = np.eye(n_states)
    for j in range(N_weight):
        W_sqrt_inv[idx_weighting, idx_weighting] = 1 / np.sqrt(w22[j])
        for i in range(N):
            velocity = np.zeros((n_states, 1))
            projector = np.eye(n_states)
            A[idx_activation, idx_activation] = a22[i]
            for x, J in zip(x_ref_list, J_list):
                velocity += get_velocity(x, velocity, J, projector, A,
                                         W_sqrt_inv, np.eye(n_task), sigma_max)
                projector = update_projector(J, projector, A, W_sqrt_inv,
                                             np.eye(n_task), sigma_max)
            # final minimization of velocity
            # U, s, V_t = np.linalg.svd(projector)
            # print(s)
            if final_velocity_minimization:
                velocity += get_velocity(np.zeros((n_states, 1)), velocity,
                                         np.eye(n_states), projector,
                                         np.eye(n_states), W_sqrt_inv, P0_sqrt,
                                         sigma_max)
            dq_w[j, i, :] = velocity.reshape(-1)
            dx_w[j, i, :] = (J_list[0] @ dq_w[j, i, :]).reshape(-1)

    # vary only joint space weighting, final velocity minimization:
    final_velocity_minimization = True
    W_sqrt_inv = np.eye(n_states)
    P0_sqrt = np.eye(n_states)
    for j in range(N_weight):
        W_sqrt_inv[idx_weighting, idx_weighting] = 1 / np.sqrt(w22[j])
        for i in range(N):
            velocity = np.zeros((n_states, 1))
            projector = np.eye(n_states)
            A[idx_activation, idx_activation] = a22[i]
            for x, J in zip(x_ref_list, J_list):
                velocity += get_velocity(x, velocity, J, projector, A,
                                         W_sqrt_inv, np.eye(n_task), sigma_max)
                projector = update_projector(J, projector, A, W_sqrt_inv,
                                             np.eye(n_task), sigma_max)
            # final minimization of velocity
            if final_velocity_minimization:
                velocity += get_velocity(np.zeros((n_states, 1)), velocity,
                                         np.eye(n_states), projector,
                                         np.eye(n_states), W_sqrt_inv, P0_sqrt,
                                         sigma_max)
            dq_p[j, i, :] = velocity.reshape(-1)
            dx_p[j, i, :] = (J_list[0] @ dq_p[j, i, :]).reshape(-1)

    # vary both weightings
    final_velocity_minimization = True
    dx_wp = np.zeros((N_weight, N, n_task))
    dq_wp = np.zeros((N_weight, N, n_states))
    W_sqrt_inv = np.eye(n_states)
    P0_sqrt = np.eye(n_states)
    for j in range(N_weight):
        P0_sqrt[idx_weighting, idx_weighting] = np.sqrt(p22[j])
        W_sqrt_inv[idx_weighting, idx_weighting] = 1 / np.sqrt(w22[j])
        for i in range(N):
            velocity = np.zeros((n_states, 1))
            projector = np.eye(n_states)
            A[idx_activation, idx_activation] = a22[i]
            for x, J in zip(x_ref_list, J_list):
                velocity += get_velocity(x, velocity, J, projector, A,
                                         W_sqrt_inv, np.eye(n_task), sigma_max)
                projector = update_projector(J, projector, A, W_sqrt_inv,
                                             np.eye(n_task), sigma_max)
            # final minimization of velocity
            if final_velocity_minimization:
                velocity += get_velocity(np.zeros((n_states, 1)), velocity,
                                         np.eye(n_states), projector,
                                         np.eye(n_states), W_sqrt_inv, P0_sqrt,
                                         sigma_max)
            dq_wp[j, i, :] = velocity.reshape(-1)
            dx_wp[j, i, :] = (J_list[0] @ dq_wp[j, i, :]).reshape(-1)

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(n_task + n_states, 3, sharex='all', sharey='row')
    axes[0, 0].set_title("Only joint velocity weighting, no final minimization")
    for j in range(N_weight):
        for i in range(n_task):
            axes[i, 0].plot(a22,
                            dx_w[j, :, i],
                            c=c_mum[j],
                            label="w22: " + str(p22[j]))
    axes[-1, 0].set_xlabel("activation value a(2,2)")
    for i in range(n_task):
        axes[i, 0].set_ylabel("task velocity dx" + str(i))
        axes[i, 0].legend()
        axes[i, 0].grid()
    for j in range(N_weight):
        for i in range(n_states):
            axes[i + n_task, 0].plot(a22,
                                     dq_w[j, :, i],
                                     c=c_mum[j],
                                     label="w22: " + str(p22[j]))
    axes[-1, 0].set_xlabel("activation value a(2,2)")
    for i in range(n_states):
        maximum = np.max(dq_w[:, :, i])
        minimum = np.min(dq_w[:, :, i])
        min_lim = minimum - (maximum - minimum) * 0.05
        max_lim = maximum + (maximum - minimum) * 0.05
        # axes[i+n_task, 0].set_ylim(min_lim, max_lim)
        axes[i + n_task, 0].set_ylabel("joint velocity dq" + str(i))
        axes[i + n_task, 0].legend()
        axes[i + n_task, 0].grid()

    axes[0, 1].set_title("Only joint velocity weighting, final minimization")
    for j in range(N_weight):
        for i in range(n_task):
            axes[i, 1].plot(a22,
                            dx_p[j, :, i],
                            c=c_mum[j],
                            label="w22: " + str(p22[j]))
    axes[-1, 1].set_xlabel("activation value a(2,2)")
    for i in range(n_task):
        maximum = np.max(dx_p[:, :, i])
        minimum = np.min(dx_p[:, :, i])
        min_lim = minimum - (maximum - minimum) * 0.05
        max_lim = maximum + (maximum - minimum) * 0.05
        # axes[i, 1].set_ylim(min_lim, max_lim)
        axes[i, 1].legend()
        axes[i, 1].grid()
    for j in range(N_weight):
        for i in range(n_states):
            axes[i + n_task, 1].plot(a22,
                                     dq_p[j, :, i],
                                     c=c_mum[j],
                                     label="w22: " + str(p22[j]))
    axes[-1, 1].set_xlabel("activation value a(2,2)")
    for i in range(n_states):
        maximum = np.max(dq_p[:, :, i])
        minimum = np.min(dq_p[:, :, i])
        min_lim = minimum - (maximum - minimum) * 0.05
        max_lim = maximum + (maximum - minimum) * 0.05
        # axes[i+n_task, 1].set_ylim(min_lim, max_lim)
        axes[i + n_task, 1].legend()
        axes[i + n_task, 1].grid()

    axes[0, 2].set_title("Both weightings with final velocity minimization")
    for j in range(N_weight):
        for i in range(n_task):
            axes[i, 2].plot(a22,
                            dx_wp[j, :, i],
                            c=c_mum[j],
                            label="w22: " + str(p22[j]))
    axes[-1, 2].set_xlabel("activation value a(2,2)")
    for j in range(N_weight):
        for i in range(n_states):
            axes[i + n_task, 2].plot(a22,
                                     dq_wp[j, :, i],
                                     c=c_mum[j],
                                     label="w22: " + str(p22[j]))
    axes[-1, 2].set_xlabel("activation value a(2,2)")
    for i in range(n_task):
        axes[i, 2].legend()
        axes[i, 2].grid()

    for i in range(n_states):
        axes[i + n_task, 2].legend()
        axes[i + n_task, 2].grid()

    for i in range(n_task):
        maximum = max(np.max(dx_w[:, :, i]),
                      max(np.max(dx_p[:, :, i]), np.max(dx_wp[:, :, i])))
        minimum = min(np.min(dx_w[:, :, i]),
                      min(np.min(dx_p[:, :, i]), np.min(dx_wp[:, :, i])))
        min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
        max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
        for j in range(3):
            axes[i, j].set_ylim(min_lim, max_lim)

    for i in range(n_states):
        maximum = max(np.max(dq_w[:, :, i]),
                      max(np.max(dq_p[:, :, i]), np.max(dq_wp[:, :, i])))
        minimum = min(np.min(dq_w[:, :, i]),
                      min(np.min(dq_p[:, :, i]), np.min(dq_wp[:, :, i])))
        min_lim = minimum - (maximum - minimum) * 0.05
        max_lim = maximum + (maximum - minimum) * 0.05
        for j in range(3):
            axes[i + n_task, j].set_ylim(min_lim, max_lim)

    # Plot with two rows, compare all three cases

    fig, axes = plt.subplots(2, 3, sharex='all', sharey='row')
    axes[0, 0].set_title("Only joint velocity weighting, no final minimization")
    for j in range(N_weight):
        axes[0, 0].plot(a22,
                        dx_w[j, :, 1],
                        c=c_mum[j],
                        label="w22: " + str(p22[j]))
    axes[0, 0].set_ylabel("task velocity dx" + str(idx_activation))
    axes[0, 0].legend()
    axes[0, 0].grid()
    for j in range(N_weight):
        axes[1, 0].plot(a22,
                        dq_w[j, :, 1],
                        c=c_mum[j],
                        label="w22: " + str(p22[j]))
    axes[1, 0].set_ylabel("joint velocity dq" + str(idx_weighting))
    axes[1, 0].legend()
    axes[1, 0].grid()
    axes[-1, 0].set_xlabel("activation value a(2,2)")

    axes[0, 1].set_title("Only joint velocity weighting, final minimization")
    for j in range(N_weight):
        axes[0, 1].plot(a22,
                        dx_p[j, :, 1],
                        c=c_mum[j],
                        label="w22: " + str(p22[j]))
    axes[0, 1].legend()
    axes[0, 1].grid()
    for j in range(N_weight):
        axes[1, 1].plot(a22,
                        dq_p[j, :, 1],
                        c=c_mum[j],
                        label="w22: " + str(p22[j]))
    axes[1, 1].legend()
    axes[1, 1].grid()
    axes[-1, 1].set_xlabel("activation value a(2,2)")

    axes[0, 2].set_title("Both weightings with final minimization")
    for j in range(N_weight):
        axes[0, 2].plot(a22,
                        dx_wp[j, :, 1],
                        c=c_mum[j],
                        label="w22: " + str(p22[j]))
    axes[0, 2].legend()
    axes[0, 2].grid()
    for j in range(N_weight):
        axes[1, 2].plot(a22,
                        dq_wp[j, :, 1],
                        c=c_mum[j],
                        label="w22: " + str(p22[j]))
    axes[1, 2].legend()
    axes[1, 2].grid()
    axes[-1, 2].set_xlabel("activation value a(2,2)")

    # single plots
    fig, axes = plt.subplots(1, 2, sharex='all')
    fig.suptitle(
        "Only joint velocity weighting, no final velocity minimization")
    for j in range(N_weight):
        axes[0].plot(a22,
                     dx_w[j, :, idx_activation],
                     c=c_mum[j],
                     label="w" + str(idx_weighting + 1) +
                     str(idx_weighting + 1) + ": " + str(p22[j]))
    axes[-1].set_xlabel("activation value a(2,2)")
    maximum = np.max(dx_w[:, :, idx_activation])
    minimum = np.min(dx_w[:, :, idx_activation])
    min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
    max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
    axes[0].set_ylim(min_lim, max_lim)
    axes[0].set_ylabel("task velocity dx" + str(idx_activation))
    axes[0].legend()
    axes[0].grid()

    for j in range(N_weight):
        axes[1].plot(a22,
                     dq_w[j, :, idx_weighting],
                     c=c_mum[j],
                     label="w" + str(idx_weighting + 1) +
                     str(idx_weighting + 1) + ": " + str(p22[j]))
    axes[1].set_xlabel("activation value a(2,2)")
    for _ in range(n_states):
        maximum = np.max(dq_w[:, :, idx_weighting])
        minimum = np.min(dq_w[:, :, idx_weighting])
        min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
        max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
        axes[1].set_ylim(min_lim, max_lim)
        axes[1].set_ylabel("joint velocity dq" + str(idx_weighting))
        axes[1].legend()
        axes[1].grid()

    fig, axes = plt.subplots(1, 2, sharex='all')
    fig.suptitle(
        "Only joint velocity weighting, with final velocity minimization")
    for j in range(N_weight):
        axes[0].plot(a22,
                     dx_p[j, :, idx_activation],
                     c=c_mum[j],
                     label="w" + str(idx_weighting + 1) +
                     str(idx_weighting + 1) + ": " + str(p22[j]))
    axes[-1].set_xlabel("activation value a(2,2)")
    maximum = np.max(dx_p[:, :, idx_activation])
    minimum = np.min(dx_p[:, :, idx_activation])
    min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
    max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
    axes[0].set_ylim(min_lim, max_lim)
    axes[0].set_ylabel("task velocity dx" + str(idx_activation))
    axes[0].legend()
    axes[0].grid()

    for j in range(N_weight):
        axes[1].plot(a22,
                     dq_p[j, :, idx_weighting],
                     c=c_mum[j],
                     label="w" + str(idx_weighting + 1) +
                     str(idx_weighting + 1) + ": " + str(p22[j]))
    axes[1].set_xlabel("activation value a(2,2)")
    for _ in range(n_states):
        maximum = np.max(dq_p[:, :, idx_weighting])
        minimum = np.min(dq_p[:, :, idx_weighting])
        min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
        max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
        axes[1].set_ylim(min_lim, max_lim)
        axes[1].set_ylabel("joint velocity dq" + str(idx_weighting))
        axes[1].legend()
        axes[1].grid()

    fig, axes = plt.subplots(1, 2, sharex='all')
    fig.suptitle("Both weightings with final velocity minimization")
    for j in range(N_weight):
        axes[0].plot(a22,
                     dx_wp[j, :, idx_activation],
                     c=c_mum[j],
                     label="w" + str(idx_weighting + 1) +
                     str(idx_weighting + 1) + ": " + str(p22[j]))
    axes[-1].set_xlabel("activation value a(2,2)")
    maximum = np.max(dx_wp[:, :, idx_activation])
    minimum = np.min(dx_wp[:, :, idx_activation])
    min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
    max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
    axes[0].set_ylim(min_lim, max_lim)
    axes[0].set_ylabel("task velocity dx" + str(idx_activation))
    axes[0].legend()
    axes[0].grid()

    for j in range(N_weight):
        axes[1].plot(a22,
                     dq_wp[j, :, idx_weighting],
                     c=c_mum[j],
                     label="w" + str(idx_weighting + 1) +
                     str(idx_weighting + 1) + ": " + str(p22[j]))
    axes[1].set_xlabel("activation value a(2,2)")
    for _ in range(n_states):
        maximum = np.max(dq_wp[:, :, idx_weighting])
        minimum = np.min(dq_wp[:, :, idx_weighting])
        min_lim = min(minimum - 0.05, minimum - (maximum - minimum) * 0.05)
        max_lim = max(maximum + 0.05, maximum + (maximum - minimum) * 0.05)
        axes[1].set_ylim(min_lim, max_lim)
        axes[1].set_ylabel("joint velocity dq" + str(idx_weighting))
        axes[1].legend()
        axes[1].grid()

    plt.show()


if __name__ == "__main__":
    main()
