import matplotlib.pyplot as plt
import numpy as np

from manipulator_kinematics import Trafo


class Viz:
    def __init__(self):
        self.fig_handle = plt.figure()
        self.axes = plt.axes(projection='3d')
        self.base_trafo = Trafo()

    def setBaseTrafo(self, tf: Trafo):
        self.base_trafo = tf

    def set_axes_equal(self):
        """
        Make axes of 3D plot have equal scale so that spheres appear as spheres,
        cubes as cubes, etc.

        Input
          ax: a matplotlib axis, e.g., as output from plt.gca().
        """

        x_limits = self.axes.get_xlim3d()
        y_limits = self.axes.get_ylim3d()
        z_limits = self.axes.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        # The plot bounding box is a sphere in the sense of the infinity
        # norm, hence I call half the max range the plot radius.
        plot_radius = 0.5 * max([x_range, y_range, z_range])

        self.axes.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        self.axes.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        self.axes.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    def plotManipulator(self, positions: np.ndarray):
        positions = self.base_trafo.transform(positions)
        self.axes.plot3D(positions[0, :-1], positions[1, :-1], positions[2, :-1], c='k')
        self.axes.plot3D(positions[0, -2:], positions[1, -2:], positions[2, -2:], c='r')
        self.axes.scatter3D(positions[0, :], positions[1, :], positions[2, :], c='b', s=5.0)
        plot_gradient = False
        if plot_gradient:
            position = positions[:, 2].reshape(-1, 1)
            ellipse_ax = 0.26 + 0.05
            ellipse_az = 0.2 + 0.05
            ellipse_cx = -0.203
            ellipse_cz = -0.131675
            position_tf = self.base_trafo.inverse_transform(position)
            jacobian = 2 * np.array([(position_tf[0][0] - ellipse_cx) / ellipse_ax ** 2, 0.0, \
                                     (position_tf[2][0] - ellipse_cz) / ellipse_az ** 2]).reshape(-1, 1)
            jacobian /= np.linalg.norm(jacobian)
            jacobian = self.base_trafo.rotate(jacobian)
            self.axes.plot3D([position[0][0], jacobian[0][0]], [position[1][0], jacobian[1][0]],
                             [position[2][0], jacobian[2][0]], c='g')
            position = positions[:, 5].reshape(-1, 1)
            position_tf = self.base_trafo.inverse_transform(position)
            jacobian = 2 * np.array([(position_tf[0][0] - ellipse_cx) / ellipse_ax ** 2, 0.0, \
                                     (position_tf[2][0] - ellipse_cz) / ellipse_az ** 2]).reshape(-1, 1)
            jacobian /= np.linalg.norm(jacobian)
            jacobian = self.base_trafo.rotate(jacobian)
            self.axes.plot3D([position[0], jacobian[0]], [position[1], jacobian[1]], [position[2], jacobian[2]], c='g')

    def create(self):
        axis_scale = 0.1
        x_axis = axis_scale * np.array([[0.0, 1.0], [0.0, 0.0], [0.0, 0.0]])
        y_axis = axis_scale * np.array([[0.0, 0.0], [0.0, 1.0], [0.0, 0.0]])
        z_axis = axis_scale * np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 1.0]])

        x_axis = self.base_trafo.transform(x_axis)
        y_axis = self.base_trafo.transform(y_axis)
        z_axis = self.base_trafo.transform(z_axis)
        self.axes.plot3D(x_axis[0, :], x_axis[1, :], x_axis[2, :], c='r')
        self.axes.plot3D(y_axis[0, :], y_axis[1, :], y_axis[2, :], c='g')
        self.axes.plot3D(z_axis[0, :], z_axis[1, :], z_axis[2, :], c='b')

        x_lim = 0.01627
        x_lim_constr = x_lim + 0.05
        z_lim = 0.0179
        z_lim_constr = z_lim + 0.05
        plane_scale = 0.3

        # horizontal restricting plane:
        coord_x = np.array([-plane_scale, x_lim])
        coord_y = np.array([-plane_scale, plane_scale])
        x, y = np.meshgrid(coord_x, coord_y)
        z = z_lim * np.ones_like(x)
        surf_points = np.vstack((np.reshape(x, -1), np.reshape(y, -1), np.reshape(z, -1)))
        surf_points = self.base_trafo.transform(surf_points)

        self.axes.plot_surface(surf_points[0, :].reshape(2, 2), surf_points[1, :].reshape(2, 2),
                               surf_points[2, :].reshape(2, 2), color='b', alpha=0.6)

        coord_x = np.array([-plane_scale, x_lim_constr])
        coord_y = np.array([-plane_scale, plane_scale])
        x, y = np.meshgrid(coord_x, coord_y)
        z = z_lim_constr * np.ones_like(x)
        surf_points = np.vstack((np.reshape(x, -1), np.reshape(y, -1), np.reshape(z, -1)))
        surf_points = self.base_trafo.transform(surf_points)

        self.axes.plot_surface(surf_points[0, :].reshape(2, 2), surf_points[1, :].reshape(2, 2),
                               surf_points[2, :].reshape(2, 2), color='b', alpha=0.4)

        # vertical restricting plane:
        coord_y = np.array([-plane_scale, plane_scale])
        coord_z = np.array([-plane_scale, z_lim])
        y, z = np.meshgrid(coord_y, coord_z)
        x = x_lim * np.ones_like(y)
        surf_points = np.vstack((np.reshape(x, -1), np.reshape(y, -1), np.reshape(z, -1)))
        surf_points = self.base_trafo.transform(surf_points)
        self.axes.plot_surface(surf_points[0, :].reshape(2, 2), surf_points[1, :].reshape(2, 2),
                               surf_points[2, :].reshape(2, 2), color='r', alpha=0.6)

        coord_y = np.array([-plane_scale, plane_scale])
        coord_z = np.array([-plane_scale, z_lim_constr])
        y, z = np.meshgrid(coord_y, coord_z)
        x = x_lim_constr * np.ones_like(y)
        surf_points = np.vstack((np.reshape(x, -1), np.reshape(y, -1), np.reshape(z, -1)))
        surf_points = self.base_trafo.transform(surf_points)
        self.axes.plot_surface(surf_points[0, :].reshape(2, 2), surf_points[1, :].reshape(2, 2),
                               surf_points[2, :].reshape(2, 2), color='r', alpha=0.4)

        # ellipse:

        ellipse_ax = 0.26 + 0.05
        ellipse_az = 0.2 + 0.05
        ellipse_cx = -0.203
        ellipse_cz = -0.131675
        N_grid = 100

        coord_y = np.linspace(-plane_scale, plane_scale, 2)
        coord_theta = np.linspace(0, 2 * np.pi, N_grid)
        y, theta = np.meshgrid(coord_y, coord_theta)

        x = (ellipse_ax) * np.cos(theta) + ellipse_cx
        z = (ellipse_az) * np.sin(theta) + ellipse_cz
        surf_points = np.vstack((np.reshape(x, -1), np.reshape(y, -1), np.reshape(z, -1)))
        surf_points = self.base_trafo.transform(surf_points)
        self.axes.plot_surface(surf_points[0, :].reshape(N_grid, 2), surf_points[1, :].reshape(N_grid, 2),
                               surf_points[2, :].reshape(N_grid, 2), color='k', alpha=0.6)

        self.set_axes_equal()
        self.axes.set_xlabel('x')
        self.axes.set_ylabel('y')
        self.axes.set_zlabel('z')
        plt.show()
