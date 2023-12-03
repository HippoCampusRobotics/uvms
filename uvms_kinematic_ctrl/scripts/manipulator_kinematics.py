from __future__ import annotations
import numpy as np

# constants
theta_c = np.arctan(40 / 145.3)
a1 = np.sqrt(40 ** 2 + 145.3 ** 2) / 1000
e3 = np.array([[0.0], [0.0], [1.0]])


def solve(manipulator: Manipulator, col_constr: [], joint_limits: [JointLimitConstraint], q0: np.ndarray,
          eps: float) -> (bool, np.ndarray):
    q = q0  # initialize

    manipulator.updateTrafos(q)
    for constr in col_constr:
        constr.update(manipulator)
    for joint_limit in joint_limits:
        joint_limit.update(q)

    constraints = joint_limits + col_constr
    f = np.array([[]])
    for constraint in constraints:
        if constraint.isActive():
            if np.size(f) > 0:
                f = np.vstack((f, constraint.getTaskError()))
            else:
                f = constraint.getTaskError()

    f = 0.0
    for constraint in constraints:
        if constraint.isActive():
            f += abs(constraint.getTaskError())
    counter = 0
    while (f > eps) & (counter < 1000):
        jacobian = np.array([[]])
        projector = np.eye(4)
        dq = np.zeros((4, 1))
        for constraint in constraints:
            if constraint.isActive():
                jacobian_tmp = constraint.getJacobian()
                if np.size(jacobian) > 0:
                    jacobian = np.vstack((jacobian, jacobian_tmp))
                else:
                    jacobian = jacobian_tmp
                dq += projector @ (jacobian_tmp.transpose() @ np.linalg.inv(
                    (jacobian_tmp @ jacobian_tmp.transpose())) @ constraint.getTaskError()).reshape(-1, 1)
                # projector = np.eye(4) - jacobian.transpose() @ np.linalg.inv((jacobian @ jacobian.transpose())) @ jacobian
                projector = projector @ (np.eye(4) - jacobian_tmp.transpose() @ np.linalg.inv(
                    (jacobian_tmp @ jacobian_tmp.transpose())) @ jacobian_tmp)
        q += dq
        manipulator.updateTrafos(q)

        for constr in col_constr:
            constr.update(manipulator)
        for joint_limit in joint_limits:
            joint_limit.update(q)

        constraints = joint_limits + col_constr
        f = np.array([[]])
        for constraint in constraints:
            if constraint.isActive():
                if np.size(f) > 0:
                    f = np.vstack((f, constraint.getTaskError()))
                else:
                    f = constraint.getTaskError()

        f = 0.0
        for constraint in constraints:
            if constraint.isActive():
                f += abs(constraint.getTaskError())
        print(np.linalg.norm(f))
        counter += 1
    if f <= eps:
        return True, q
    else:
        return False, q


class JointLimitConstraint:
    def __init__(self, q_low: float, q_up: float, delta: float, idx: int):
        self.low = q_low
        self.up = q_up
        self.delta = delta
        self.active = False
        self.error = np.array([[0.0]])
        self.jacobian = np.array([[0.0, 0.0, 0.0, 0.0]])
        self.jacobian[0][idx] = 1.0
        self.idx = idx

    def update(self, q: np.ndarray):
        if (q[self.idx] >= self.up - self.delta):
            self.active = True
            self.error = np.array([[self.up - self.delta - q[self.idx]]]).reshape(-1, 1)
        elif (q[self.idx] <= self.low + self.delta):
            self.active = True
            self.error = np.array([[self.low + self.delta - q[self.idx]]]).reshape(-1, 1)
        else:
            self.active = False

    def getJacobian(self) -> np.ndarray:
        return self.jacobian

    def getTaskError(self):
        return self.error

    def isActive(self) -> bool:
        return self.active


class PlaneConstraint:
    def __init__(self, n: [np.ndarray], p: [np.ndarray], link_idx: int):
        assert (len(n) == len(p))
        self.n = n
        self.plane_index = 0
        self.p = p
        self.error = np.array([[0.0]])
        self.active = False
        self.link_idx = link_idx
        self.jacobian = np.zeros((1, 4))

    def isActive(self):
        return self.active

    def getTaskError(self) -> float:
        return self.error

    def getDistance(self, p: np.ndarray, idx) -> float:

        return (self.n[idx].transpose() @ (p - self.p[idx])).item()

    def inCollision(self, p: np.ndarray) -> bool:
        for i in range(len(self.n)):
            if self.getDistance(p, i) >= 0:
                return False
        return True

    def update(self, manipulator: Manipulator):
        position = manipulator.getLinkPosition(self.link_idx).reshape(-1, 1)
        J_tmp = np.zeros((3, 4))
        manipulator.getLinkPositionJacobian(J_tmp, self.link_idx)
        self.jacobian = self.n[self.plane_index].transpose() @ J_tmp
        self.error = np.array([[np.inf]])

        if self.inCollision(position):
            self.active = True
            for j in range(len(self.n)):
                tmp_distance = self.getDistance(position, j)
                if -tmp_distance < self.error.item():
                    self.plane_index = j
                    self.error = -np.array([[tmp_distance]])
        else:
            self.active = False

    def getJacobian(self):
        return self.jacobian


class EllipseConstraint:
    def __init__(self, ax: float, az: float, cx: float, cz: float, link_idx: int):
        self.ax = ax
        self.az = az
        self.cx = cx
        self.cz = cz
        self.error = np.array([[0.0]])
        self.active = False
        self.link_idx = link_idx
        self.jacobian = np.zeros((1, 4))

    def isActive(self):
        return self.active

    def getTaskError(self) -> float:
        return self.error

    def getDistance(self, p: np.ndarray) -> float:

        return (p[0] - self.cx) ** 2 / self.ax ** 2 + (p[2] - self.cz) ** 2 / self.az ** 2 - 1

    def inCollision(self, p: np.ndarray) -> bool:
        if self.getDistance(p) >= 0:
            return False
        return True

    def update(self, manipulator: Manipulator):
        position = manipulator.getLinkPosition(self.link_idx).reshape(-1, 1)
        J_tmp = np.zeros((3, 4))
        manipulator.getLinkPositionJacobian(J_tmp, self.link_idx)
        self.jacobian = 2 * ((position[0] - self.cx) / self.ax ** 2 * J_tmp[0, :].reshape(1, -1) +
                             (position[2] - self.cz) / self.az ** 2 * J_tmp[2, :].reshape(1, -1))
        self.error = np.array([[np.inf]])

        if self.inCollision(position):
            self.active = True
            self.error = self.getDistance(position)
            print(self.error)
        else:
            self.active = False

    def getJacobian(self):
        return self.jacobian


class Trafo:
    def __init__(self, R=np.zeros((3, 3)), p=np.zeros((3, 1))):
        self.R = R
        self.p = p

    def fromDH(self, d, theta, a, alpha):
        self.p = np.array([[a * np.cos(theta)],
                           [a * np.sin(theta)],
                           [d]])
        self.R = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha)],
                           [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha)],
                           [0, np.sin(alpha), np.cos(alpha)]])

    def fromRPY(self, vec: np.ndarray, r, p, y):
        self.p = vec
        self.R = np.array([[np.cos(y) * np.cos(p), np.cos(y) * np.sin(p) * np.sin(r) - np.cos(r) * np.sin(y),
                            np.sin(y) * np.sin(r) + np.cos(y) * np.cos(r) * np.sin(p)],
                           [np.cos(p) * np.sin(y), np.cos(y) * np.cos(r) + np.sin(y) * np.sin(p) * np.sin(r),
                            np.cos(r) * np.sin(y) * np.sin(p) - np.cos(y) * np.sin(r)],
                           [- np.sin(p), np.cos(p) * np.sin(r), np.cos(p) * np.cos(r)]])

    def transform(self, vec: np.ndarray) -> np.ndarray:
        return self.R @ vec + self.p

    def inverse_transform(self, vec: np.ndarray) -> np.ndarray:
        return self.R.transpose() @ (vec - self.p)

    def rotate(self, vec: np.ndarray) -> np.ndarray:
        return self.R @ vec

    def rotate_backwards(self, vec: np.ndarray) -> np.ndarray:
        return self.R.transpose() @ vec

    def matrix(self):
        return np.vstack((np.hstack((self.R, self.p)), np.array([[0.0, 0.0, 0.0, 1.0]])))

    def __rmul__(self, other: Trafo) -> Trafo:
        return Trafo(other.R @ self.R, other.R @ self.p + other.p)

    def __mul__(self, other: Trafo) -> Trafo:
        return Trafo(self.R @ other.R, self.R @ other.p + self.p)


class Manipulator:
    def __init__(self):
        self.tfs = [Trafo() for i in range(6)]

    def updateTrafos(self, q: np.ndarray):

        assert len(self.tfs) == 6
        assert np.shape(q)[0] == 4
        self.tfs[0].fromDH(0.0462, q[0][0] + np.pi, 0.02, np.pi / 2)
        self.tfs[1].fromDH(0, q[1][0] + theta_c - np.pi / 2, a1, np.pi)
        self.tfs[2].fromDH(0, q[2][0] + theta_c - np.pi / 2, 0.02, -np.pi / 2)
        self.tfs[3].fromDH(-0.18, q[3][0] + np.pi / 2, 0, np.pi / 2)
        self.tfs[4].fromDH(0, -np.pi / 2, 0, 0)
        self.tfs[5].fromDH(0, 0.0, 0.05, 0)  # additional transformation not part of the kinematics but represents the
        #    remaining length of the endeffector

        for i in range(1, len(self.tfs)):
            self.tfs[i] = self.tfs[i - 1] * self.tfs[i]

    def getEefPosition(self):
        return self.tfs[-1].matrix()[0:3, 3].reshape(3, 1)

    def getEefPositionJacobian(self, J: np.ndarray):
        for i in range(0, 4):
            if i == 0:

                J[:, i] = np.cross(e3, self.tfs[-1].matrix()[0:3, 3].reshape(3, 1), axis=0).reshape(-1)
            else:
                J[:, i] = np.cross(self.tfs[i - 1].matrix()[0:3, 0:3] @ e3,
                                   self.tfs[-1].matrix()[0:3, 3].reshape(3, 1) - \
                                   self.tfs[i - 1].matrix()[0:3, 3].reshape(3, 1), axis=0).reshape(-1)

    # idx 0 => link 1
    def getLinkPositionJacobian(self, J: np.ndarray, idx):
        for i in range(0, 4):
            if i == 0:
                J[:, i] = np.cross(e3, self.tfs[idx].matrix()[0:3, 3].reshape(3, 1), axis=0).reshape(-1)
            elif i <= idx:  # axis x position in world coordinate system
                J[:, i] = np.cross(self.tfs[i - 1].matrix()[0:3, 0:3] @ e3,
                                   self.tfs[idx].matrix()[0:3, 3].reshape(3, 1) - \
                                   self.tfs[i - 1].matrix()[0:3, 3].reshape(3, 1), axis=0).reshape(-1)
            else:
                J[:, i] = np.zeros(3, )

    # idx 0 => link 1
    def getLinkPosition(self, idx: int):
        return self.tfs[idx].matrix()[0:3, 3]

    # idx 0 => link 1
    def getLinkPositions(self):
        x = np.zeros((3, len(self.tfs) + 1))
        x[:, 0] = np.zeros((3,))
        for i in range(len(self.tfs)):
            x[:, i + 1] = self.tfs[i].matrix()[0:3, 3]
        return x
