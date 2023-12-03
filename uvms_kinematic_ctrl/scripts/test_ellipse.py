import numpy as np

a_x = 2.0
a_z = 1.0
c_x = 0.3
c_z = -0.1
delta = 0.4

eps = 1e-12


def polinom_4(x, a, b, c, d) -> float:
    return x * (x * (x * (x + a) + b) + c) + d


def solveP3(x: np.ndarray, a: float, b: float, c: float) -> int:
    a2 = a * a
    q = (a2 - 3 * b) / 9
    r = (a * (2 * a2 - 9 * b) + 27 * c) / 54
    r2 = r * r
    q3 = q * q * q
    A = float()
    B = float()
    if (r2 < q3):
        t = r / np.sqrt(q3)
        if (t < -1):
            t = -1
        if (t > 1):
            t = 1
        t = np.arccos(t)
        a /= 3
        q = -2 * np.sqrt(q)
        x[0] = q * np.cos(t / 3) - a
        x[1] = q * np.cos((t + 2 * np.pi) / 3) - a
        x[2] = q * np.cos((t - 2 * np.pi) / 3) - a
        return 3

    else:
        A = -pow(np.abs(r) + np.sqrt(r2 - q3), 1. / 3)
        if (r < 0):
            A = -A
        if (0 == A):
            B = 0
        else:
            B = q / A

        a /= 3
        x[0] = (A + B) - a
        x[1] = -0.5 * (A + B) - a
        x[2] = 0.5 * np.sqrt(3.) * (A - B)
        if (np.abs(x[2]) < eps):
            x[2] = x[1]
            return 2

        return 1


def solve_quartic(a, b, c, d) -> np.ndarray:
    a3 = -b
    b3 = a * c - 4. * d
    c3 = -a * a * d - c * c + 4.0 * b * d

    x3 = np.zeros((3,))
    iZeroes = solveP3(x3, a3, b3, c3)

    q1 = float()
    q2 = float()
    p1 = float()
    p2 = float()
    D = float()
    sqD = float()
    y = float()

    y = x3[0]
    if (iZeroes != 1):
        if (np.abs(x3[1]) > np.abs(y)):
            y = x3[1]
        if (np.abs(x3[2]) > np.abs(y)):
            y = x3[2]

    D = y * y - 4 * d
    if (np.abs(D) < eps):

        q1 = q2 = y * 0.5
        D = a * a - 4 * (b - y)
        if (np.abs(D) < eps):
            p1 = p2 = a * 0.5

        else:
            sqD = np.sqrt(D)
            p1 = (a + sqD) * 0.5
            p2 = (a - sqD) * 0.5

    else:
        sqD = np.sqrt(D)
        q1 = (y + sqD) * 0.5
        q2 = (y - sqD) * 0.5
        p1 = (a * q1 - c) / (q1 - q2)
        p2 = (c - a * q2) / (q1 - q2)

    retval = np.zeros((4,))
    counter = 0
    D = p1 * p1 - 4 * q1
    if (D >= 0.0):
        sqD = np.sqrt(D)
        retval[counter] = (-p1 + sqD) * 0.5
        counter += 1
        retval[counter] = (-p1 - sqD) * 0.5
        counter += 1

    D = p2 * p2 - 4 * q2
    if (D >= 0.0):
        sqD = np.sqrt(D)
        retval[counter] = (-p2 + sqD) * 0.5
        counter += 1
        retval[counter] = (-p2 - sqD) * 0.5
        counter += 1
    retval = retval[0:counter]
    return retval


def find_nearest_point(x, z) -> (float, float, float):
    x_centered = x - c_x
    z_centered = z - c_z
    # polynomial coefficients for x^4 + a*x^3 + b*x^2 + c*x + d
    a = 2 * a_x ** 2 + 2 * a_z ** 2
    b = a_x ** 4 + 4 * a_x ** 2 * a_z ** 2 - a_x ** 2 * x_centered ** 2 + a_z ** 4 - a_z ** 2 * z_centered ** 2
    c = 2 * a_x ** 4 * a_z ** 2 + 2 * a_x ** 2 * a_z ** 4 - 2 * a_x ** 2 * a_z ** 2 * x_centered ** 2 - 2 * a_x ** 2 * a_z ** 2 * z_centered ** 2
    d = a_x ** 4 * a_z ** 4 - a_x ** 4 * a_z ** 2 * z_centered ** 2 - a_x ** 2 * a_z ** 4 * x_centered ** 2
    roots = solve_quartic(a, b, c, d)
    t = np.max(roots)
    e_x = a_x ** 2 * x_centered / (t + a_x ** 2) + c_x
    e_z = a_z ** 2 * z_centered / (t + a_z ** 2) + c_z
    normal_x = (e_x - c_x) / a_x ** 2  # calculate gradient
    normal_z = (e_z - c_z) / a_z ** 2
    gradient_norm = np.sqrt(normal_x ** 2 + normal_z ** 2)
    dist = t * gradient_norm
    normal_x /= gradient_norm
    normal_z /= gradient_norm
    return e_x, e_z, dist, a, b, c, d, normal_x, normal_z


def main():
    # test quartic:
    n_quartic = 10000
    x_vec = np.linspace(-6, 6, n_quartic)
    a = -5.0
    b = 1.0
    c = 5.0
    d = -0.1
    roots = solve_quartic(a, b, c, d)

    y_vec = np.zeros_like(x_vec)
    for i in range(len(x_vec)):
        y_vec[i] = polinom_4(x_vec[i], a, b, c, d)

    y_roots = np.array([polinom_4(root, a, b, c, d) for root in roots])

    n = 100
    points_x = np.random.uniform(-3, 3, n)
    points_y = np.random.uniform(-3, 3, n)
    points = np.vstack((points_x, points_y))
    e_points = np.zeros_like(points)
    gradients = np.zeros_like(points)
    vectors = np.zeros_like(points)
    dists = np.zeros((n,))

    for i in range(n):
        e_points[0, i], e_points[1, i], dists[i], a, b, c, d, gradients[0, i], gradients[1, i] = find_nearest_point(
            points[0, i], points[1, i])
        vectors[:, i] = dists[i] * gradients[:, i]
        print("dist: ", dists[i])
    roots = solve_quartic(a, b, c, d)

    y_vec = np.zeros_like(x_vec)
    for i in range(len(x_vec)):
        y_vec[i] = polinom_4(x_vec[i], a, b, c, d)

    y_roots = np.array([polinom_4(root, a, b, c, d) for root in roots])

    n_ell = 100
    t = np.linspace(0, 2 * np.pi, n_ell)
    ell = np.zeros((2, n_ell))
    ell[0, :] = a_x * np.cos(t) + c_x
    ell[1, :] = a_z * np.sin(t) + c_z
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(x_vec, y_vec, label="polynomial")
    plt.scatter(roots, y_roots, label="roots")
    plt.grid()

    plt.figure()
    plt.scatter(points[0, :], points[1, :], c=np.arange(n))
    plt.gca().set_prop_cycle(None)
    plt.scatter(e_points[0, :], e_points[1, :], c=np.arange(n))
    for i in range(n):
        plt.plot([e_points[0, i], e_points[0, i] + vectors[0, i]], [e_points[1, i], e_points[1, i] + vectors[1, i]],
                 c='k')
        plt.plot([e_points[0, i], e_points[0, i] + gradients[0, i]], [e_points[1, i], e_points[1, i] + gradients[1, i]],
                 c='r')
    plt.plot(ell[0, :], ell[1, :])
    plt.gca().set_aspect("equal")
    plt.grid()

    plt.show()


if __name__ == "__main__":
    main()
