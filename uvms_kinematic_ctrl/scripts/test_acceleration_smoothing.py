import numpy as np
import matplotlib.pyplot as plt

def smooth(delta):
    max_fac = 0.95
    min_fac = 0.4
    min_acc = 0.3
    max_acc = 2.0
    out = max_fac * (1 + (max_fac - min_fac) / min_fac * min((delta - min_acc) / (max_acc - min_acc), 1)) * min_fac / max_fac * int(delta > min_acc)
    return out

def main():
    n = 100
    deltas = np.linspace(0.0, 3.0, n)
    factors = np.zeros((n, ))
    for i in range(n):
        factors[i] = smooth(deltas[i])

    plt.figure()
    plt.plot(deltas, factors)
    plt.show()





if __name__ == "__main__":
    main()


