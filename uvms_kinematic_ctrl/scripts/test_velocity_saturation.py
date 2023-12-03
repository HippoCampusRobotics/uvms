import numpy as np


def main():
    zeta_high_ = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.6, 0.6, 0.6, 0.9])
    zeta_low_ = -zeta_high_
    zeta_high = zeta_high_
    zeta_low = zeta_low_
    out_vel = np.zeros((10,))
    for task in range(5):
        scaling = 1.0
        task_cmd_vel = np.random.uniform(-2, 2, (10,))
        print(task_cmd_vel)

        for i in range(10):
            if (task_cmd_vel[i] > zeta_high[i]):
                scaling_tmp = zeta_high[i] / task_cmd_vel[i]
                if (scaling_tmp < 0) | (scaling_tmp > 1.0):
                    scaling = 0.0
                    break
                scaling = min(scaling_tmp, scaling)

            elif (task_cmd_vel[i] < zeta_low[i]):
                scaling_tmp = zeta_low[i] / task_cmd_vel[i]
                if (scaling_tmp < 0) | (scaling_tmp > 1.0):
                    scaling = 0.0
                    break
                scaling = min(scaling_tmp, scaling)
        print(scaling)
        zeta_low = zeta_low - scaling * task_cmd_vel
        zeta_high = zeta_high - scaling * task_cmd_vel
        out_vel += scaling * task_cmd_vel
    print(out_vel)
    print(np.all(zeta_high_ - out_vel >= 0))
    print(np.all(zeta_low_ - out_vel <= 0))


if __name__ == "__main__":
    main()
