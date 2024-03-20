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


def sigmoid(x, x_min, buffer):
    if x < x_min:
        return 1.0
    elif x > x_min + buffer:
        return 0.0
    else:
        return 0.5 * (np.cos((x - x_min) * np.pi / buffer) + 1.0)


def main():
    x = np.linspace(0.0, 1.0, 200)
    x_min = 0.2
    x_max = 0.8
    buffer = 0.2

    x_vec_min = np.array([sigmoid(x[i], x_min, buffer) for i in range(len(x))])
    x_vec_max = np.array(
        [sigmoid(-x[i], -x_max, buffer) for i in range(len(x))])
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(x, x_vec_min, c=c_mum[0], label=r"$a_\mathrm{min}$")
    plt.plot(x, x_vec_max, c=c_mum[1], label=r"$a_\mathrm{max}$")
    plt.vlines([x_min, x_min + buffer, x_max - buffer, x_max],
               -0.2,
               1.2,
               color="k",
               linestyles="--")
    plt.xlabel('$x$')
    plt.ylabel('activation value')
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(x, np.maximum(x_vec_min, x_vec_max), c=c_mum[0])
    plt.vlines([x_min, x_min + buffer, x_max - buffer, x_max],
               -0.2,
               1.2,
               color="k",
               linestyles="--")
    plt.xlabel('$x$')
    plt.ylabel('activation value')
    plt.legend()
    plt.grid()

    plt.show()


if __name__ == "__main__":
    main()
