import numpy as np
from manipulator_kinematics import Trafo, Manipulator, JointLimitConstraint, PlaneConstraint, EllipseConstraint, solve
from visualization import Viz
plane_x_lim = 0.01627
plane_z_lim = 0.0179
constr_x_lim = plane_x_lim + 0.05
constr_z_lim = plane_z_lim + 0.05

ellipse_ax = 0.26 + 0.05
ellipse_az = 0.2 + 0.05
ellipse_cx = -0.203
ellipse_cz = -0.131675
def main():
    plot_samples = False
    use_ellipse = True
    unlimited = 10000
    q_lim = np.array([[0.032, 6.02],
                        [0.0174533, 3.40339],
                        [0.0174533, 3.40339],
                        [-unlimited, unlimited],
                        #[-0.05, 2 * np.pi+0.05],
                        ])
                        #[0.0013, 0.0133]])

    base_tf = Trafo()
    base_tf.fromRPY(np.array([[0.0], [0.0], [0.0]]), 3.141592654, 0.0, 0.0)
    q = np.array([[0.3], [0.4], [0.8], [0.9]])

    p_vert = np.array([[constr_x_lim], [0.0], [0.0]])
    n_vert = np.array([[1.0], [0.0], [0.0]])
    p_horz = np.array([[0.0], [0.0], [constr_z_lim]])
    n_horz = np.array([[0.0], [0.0], [1.0]])
    p = [p_vert, p_horz]
    n = [n_vert, n_horz]
    if use_ellipse:
        coll_constraints = [EllipseConstraint(ellipse_ax, ellipse_az, ellipse_cx, ellipse_cz, 1), EllipseConstraint(ellipse_ax, ellipse_az, ellipse_cx, ellipse_cz, 4)]
    else:
        coll_constraints = [PlaneConstraint(n, p, 1), PlaneConstraint(n, p, 4)]
    joint_limit_constraints = [JointLimitConstraint(q_lim[i][0], q_lim[i][1], 0.1, i) for i in range(np.shape(q_lim)[0])]
    if plot_samples:
        n_samples = 300
        viz = Viz()
        viz.setBaseTrafo(base_tf)
        for n in range(n_samples):
            for i in range(np.size(q)):
                q[i] = np.random.uniform(q_lim[i, 0], q_lim[i, 1])

            manipulator = Manipulator()
            manipulator.updateTrafos(q)
            for coll_constraint in coll_constraints:
                coll_constraint.update(manipulator)
            if not np.any([coll_constraint.isActive() for coll_constraint in coll_constraints]):
                positions = manipulator.getLinkPositions()
                viz.plotManipulator(positions)

        viz.create()
    else:
        while True:
            for i in range(np.size(q)):
                q[i] = np.random.uniform(q_lim[i, 0], q_lim[i, 1])

            manipulator = Manipulator()
            manipulator.updateTrafos(q)
            for coll_constraint in coll_constraints:
                coll_constraint.update(manipulator)
            if np.any([coll_constraint.isActive() for coll_constraint in coll_constraints]):
                viz = Viz()
                viz.setBaseTrafo(base_tf)
                positions = manipulator.getLinkPositions()
                viz.plotManipulator(positions)
                print("q before: ", q)
                _, q = solve(manipulator, coll_constraints, joint_limit_constraints, q, 1e-5)
                print("q after: ", q)
                manipulator.updateTrafos(q)
                positions = manipulator.getLinkPositions()
                viz.plotManipulator(positions)
                viz.create()








if __name__ == "__main__":
    main()