import numpy as np
from scipy.optimize import fsolve

l1 = 0.2
l2 = 0.1


def bell_function(sigma, v, sigma0):
    if sigma < sigma0:
        return v * (1 - sigma / sigma0) ** 2
    else:
        return 0


def get_pos_link1(q):
    return np.array([l1 * np.cos(q[0]), l1 * np.sin(q[0])])


def get_pos(q):
    return np.array(
        [
            l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1]),
            l1 * np.sin(q[0]) + l2 * np.sin(q[0] + q[1]),
        ]
    )


def get_jacobian(q):
    return np.array(
        [
            [
                -l1 * np.sin(q[0]) - l2 * np.sin(q[0] + q[1]),
                -l2 * np.sin(q[0] + q[1]),
            ],
            [
                l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1]),
                l2 * np.cos(q[0] + q[1]),
            ],
        ]
    )


def main():
    eps = 1e-10
    T = 0.5  # s
    N = 1001
    dt = T / (N - 1)
    t = np.linspace(0, T, N)
    start = np.array([l1 - l2 + eps, l2 * 0.1])
    end = np.array([l1 - l2 + eps, -l2 * 0.1])
    p_traj = np.vstack(
        (np.linspace(start[0], end[0], N), np.linspace(start[1], end[1], N))
    )
    v_traj = np.repeat((end - start).reshape(-1, 1) / T, N, axis=1)

    def fh(x):
        return get_pos(x) - start

    x0 = fsolve(fh, np.array([np.pi / 4, np.pi / 2]))
    # unfiltered
    x = np.zeros((2, N))
    x[:, 0] = x0
    dx = np.zeros((2, N))
    dx[:, 0] = 0.0
    p = np.zeros((2, N))
    p[:, 0] = get_pos(x[:, 0])
    s_vec = np.zeros((2, N))
    # filtered:
    x_filtered = np.zeros((2, N))
    x_filtered[:, 0] = x0
    dx_filtered = np.zeros((2, N))
    dx_filtered[:, 0] = 0.0
    p_filtered = np.zeros((2, N))
    p_filtered[:, 0] = get_pos(x_filtered[:, 0])
    s_vec_filtered = np.zeros((2, N))
    p_vec_filtered = np.zeros((2, N))
    for i in range(1, N):
        J = get_jacobian(x[:, i - 1])
        dx[:, i] = (
            J.transpose()
            @ np.linalg.inv(J @ J.transpose())
            @ (
                v_traj[:, i - 1] + p_traj[:, i - 1] - get_pos(x[:, i - 1])
            ).reshape(-1, 1)
        ).reshape(-1)
        x[:, i] = x[:, i - 1] + dt * dx[:, i]
        p[:, i] = get_pos(x[:, i])

        U, s, V_t = np.linalg.svd(J)
        np.diag(s)
        V_t.transpose()
        s_vec[:, i] = s

        J_filtered = get_jacobian(x_filtered[:, i - 1])
        U_filtered, s_filtered, V_t_filtered = np.linalg.svd(J_filtered)
        np.diag(s_filtered)
        V_filtered = V_t_filtered.transpose()
        s_vec_filtered[:, i] = s_filtered
        sigma0 = 0.01
        v_smooth = 0.01
        P = np.diag(
            [
                bell_function(s_filtered[0], v_smooth, sigma0),
                bell_function(s_filtered[1], v_smooth, sigma0),
            ]
        )
        # P = np.zeros((2, 2))
        p_vec_filtered[:, i] = np.array([P[0][0], P[1][1]])
        print(P)
        dx_filtered[:, i] = (
            np.linalg.pinv(
                J_filtered.transpose() @ J_filtered
                + V_filtered @ P @ V_filtered.transpose()
            )
            @ J_filtered.transpose()
            @ (
                v_traj[:, i - 1]
                + p_traj[:, i - 1]
                - get_pos(x_filtered[:, i - 1])
            ).reshape(-1, 1)
        ).reshape(-1)
        x_filtered[:, i] = x_filtered[:, i - 1] + dt * dx_filtered[:, i]
        p_filtered[:, i] = get_pos(x_filtered[:, i])

    import matplotlib.pyplot as plt

    plt.figure()
    plt.plot(p_traj[0, :], p_traj[1, :], label='traj')
    plt.plot(p[0, :], p[1, :], label='tracking')
    plt.plot(p_filtered[0, :], p_filtered[1, :], label='filtered')
    for i in range(N):
        if i % int(N / 5) == 0:
            link_1 = get_pos_link1(x[:, i])
            link_2 = get_pos(x[:, i])
            plt.plot([0, link_1[0]], [0, link_1[1]], 'k')
            plt.plot([link_1[0], link_2[0]], [link_1[1], link_2[1]], 'k')
            link_1 = get_pos_link1(x_filtered[:, i])
            link_2 = get_pos(x_filtered[:, i])
            plt.plot([0, link_1[0]], [0, link_1[1]], 'y')
            plt.plot([link_1[0], link_2[0]], [link_1[1], link_2[1]], 'y')
    plt.gca().set_aspect('equal', 'box')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(t, p[0], label='p1')
    plt.plot(t, p[1], label='p2')
    plt.plot(t, p_filtered[0], label='p1_filtered')
    plt.plot(t, p_filtered[1], label='p2_filtered')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(t, x[0], label='x1')
    plt.plot(t, x[1], label='x2')
    plt.plot(t, x_filtered[0], label='x1_filtered')
    plt.plot(t, x_filtered[1], label='x2_filtered')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(t[1:], dx[0, 1:], label='dx1')
    plt.plot(t[1:], dx[1, 1:], label='dx2')
    plt.plot(t[1:], dx_filtered[0, 1:], label='dx1_filtered')
    plt.plot(t[1:], dx_filtered[1, 1:], label='dx2_filtered')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.title('Evolution singular values')
    plt.plot(t[1:], s_vec[0, 1:], label='s1')
    plt.plot(t[1:], s_vec[1, 1:], label='s2')
    plt.plot(t[1:], s_vec_filtered[0, 1:], label='s1_filtered')
    plt.plot(t[1:], s_vec_filtered[1, 1:], label='s2_filtered')
    plt.plot(t[1:], p_vec_filtered[0, 1:], label='bell_support1')
    plt.plot(t[1:], p_vec_filtered[1, 1:], label='bell_support2')
    plt.legend()
    plt.grid()

    v_smooth = np.linspace(0, 1, 10)
    sigma = np.logspace(-8, -1, 1000)
    sigma0 = 1e-4
    plt.figure()
    plt.title('Vary smooth fac')
    for v_smooth_fac in v_smooth:
        plt.semilogx(
            sigma,
            [
                bell_function(sigma[i], v_smooth_fac, sigma0)
                for i in range(len(sigma))
            ],
            label=str(v_smooth_fac),
        )
    plt.legend()
    plt.grid()

    sigma0 = np.logspace(-5, -1, 10)
    plt.figure()
    plt.title('Vary sigma0')
    for sigma0_fac in sigma0:
        plt.semilogx(
            sigma,
            [
                bell_function(sigma[i], 0.4, sigma0_fac)
                for i in range(len(sigma))
            ],
            label=str(sigma0_fac),
        )
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()
