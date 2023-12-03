import numpy as np


def main():
    projector = np.array([[0.00304075, -0.0361599, 0.0499732, 0.0227188, 0.0469711, -0.0490328, -0.00221098,
                           -0.000655459, 5.02166e-05, -0.00145076],
                          [0.000835974, 0.0742727, 0.0264902, -0.0704842, -0.0224563, -0.111651, 0.000129111,
                           0.00623284, 0.00633517, 0.00278534],
                          [-0.0198957, -0.0433066, 0.190791, -0.0477467, 0.00191251, -0.0874047, -0.000350082,
                           0.00949991, 0.00988759, 0.00164848],
                          [0.00914844, -0.0766016, -0.0198298, 0.188514, 0.000219906, -0.145048, -0.00851233,
                           -0.00366056, -0.00504673, -0.00661671],
                          [0.0354697, -0.101092, 0.0635728, 0.0613923, 0.264065, -0.0745137, -0.00643354, -0.0128892,
                           -0.00843282, -0.00582892],
                          [-0.0572692, -0.139523, -0.0465533, -0.150144, -0.0647486, 0.898567, 0.0232027, -0.00965536,
                           -0.00975179, 0.00616866],
                          [-0.0918106, 0.0340901, -0.0545705, -0.466812, -0.22091, 1.15832, 0.0396626, 0.00339656,
                           0.00273237, 0.0194517],
                          [-0.21819, -0.0805217, 1.13619, -0.395965, -0.630901, -0.474388, 0.00973956, 0.0946087,
                           0.0858, 0.0226721],
                          [-0.188593, -0.146244, 1.20259, -0.408412, -0.385525, -0.490165, 0.00689823, 0.0839776,
                           0.0798224, 0.0196938],
                          [-0.04073, 0.204718, -0.00945362, -0.373403, -0.18414, 0.306289, 0.0194088, 0.0153904,
                           0.0147209, 0.0156629]])

    # projector = np.random.uniform(-1, 1, size=(10,10))
    U, sigma, V_t = np.linalg.svd(projector)
    task_weighting = np.diag(np.array([100, 100, 100, 100, 100, 100, 1.0, 1.0, 1.0, 1.0]))

    velocities = np.ones((10, 1))
    I = np.eye(10)
    z_no_task_weight = - projector @ np.linalg.pinv(
        projector.transpose() @ projector + 1.0 * (I - projector).transpose() @ (
                    I - projector)) @ projector.transpose() @ velocities
    z_task_weight = - projector @ np.linalg.pinv(
        projector.transpose() @ task_weighting @ task_weighting @ projector + 1.0 * (
                    I - projector).transpose() @ task_weighting @ task_weighting @ (
                    I - projector)) @ projector.transpose() @ task_weighting @ task_weighting @ velocities
    velocities_no_task_weight = (I - projector @ np.linalg.pinv(
        projector.transpose() @ projector + 1.0 * (I - projector).transpose() @ (
                    I - projector)) @ projector.transpose()) @ velocities
    velocities_task_weight = (I - projector @ np.linalg.pinv(
        projector.transpose() @ task_weighting @ task_weighting @ projector + 1.0 * (
                    I - projector).transpose() @ task_weighting @ task_weighting @ (
                    I - projector)) @ projector.transpose() @ task_weighting @ task_weighting) @ velocities
    print("Velocities no task_weight: ", velocities_no_task_weight)
    print("Velocities task_weight: ", velocities_task_weight)
    print("Norm no task_weight: ", np.linalg.norm(velocities_no_task_weight))
    print("Norm task_weight: ", np.linalg.norm(velocities_task_weight))
    task_weighting *= 10
    velocities_task_weight_mul = (I - projector @ np.linalg.pinv(
        projector.transpose() @ task_weighting @ task_weighting @ projector + 1.0 * (
                    I - projector).transpose() @ task_weighting @ task_weighting @ (
                    I - projector)) @ projector.transpose() @ task_weighting @ task_weighting) @ velocities
    print("Norm task_weight_mul: ", np.linalg.norm(velocities_task_weight_mul))

    joint_weighting = np.diag(np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 4.7, 4.7, 4.7, 4.7]))
    velocities_no_joint_weight = (I - projector @ np.linalg.pinv(
        projector.transpose() @ projector + 1.0 * (I - projector).transpose() @ (
                    I - projector)) @ projector.transpose()) @ velocities
    velocities_joint_weight = (I - projector @ joint_weighting @ np.linalg.pinv(
        joint_weighting @ projector.transpose() @ projector @ joint_weighting + 1.0 * joint_weighting @ (
                    I - projector).transpose() @ (
                    I - projector) @ joint_weighting) @ joint_weighting @ projector.transpose()) @ velocities
    print("Velocities no joint_weight: ", velocities_no_joint_weight)
    print("Velocities joint_weight: ", velocities_joint_weight)
    print("Norm no joint_weight: ", np.linalg.norm(velocities_no_joint_weight))
    print("Norm joint_weight: ", np.linalg.norm(velocities_joint_weight))
    joint_weighting *= 10
    velocities_joint_weight_mul = (I - projector @ joint_weighting @ np.linalg.pinv(
        joint_weighting @ projector.transpose() @ projector @ joint_weighting + 1.0 * joint_weighting @ (
                    I - projector).transpose() @ (
                    I - projector) @ joint_weighting) @ joint_weighting @ projector.transpose()) @ velocities
    print("Norm joint_weight_mul: ", np.linalg.norm(velocities_joint_weight_mul))

    print("Singular values before: ", sigma)
    sigma[4:] = 0.0
    projector = U @ np.diag(sigma) @ V_t
    print("Singular values: ", sigma)
    print("Rank: ", np.linalg.matrix_rank(projector))
    weighting = np.diag(np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 4.7, 4.7, 4.7, 4.7]))
    print("Inverse: ", np.linalg.pinv(projector))
    print("Inverse weighted = : ", weighting @ np.linalg.pinv(projector @ weighting))
    print("Difference: ", np.divide(np.linalg.pinv(projector) - weighting @ np.linalg.pinv(projector @ weighting),
                                    np.linalg.pinv(projector)))

    print("out without: ", np.linalg.pinv(projector) @ np.ones((10, 1)))
    print("out with: ", weighting @ np.linalg.pinv(projector @ weighting) @ np.ones((10, 1)))
    projector = projector @ (I - np.linalg.pinv(projector) @ projector)
    print("Rank after: ", np.linalg.matrix_rank(projector))


if __name__ == "__main__":
    main()
