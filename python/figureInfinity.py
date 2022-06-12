#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting
from matplotlib import animation


class FigureInfinity():
    """figure lemniscate of Gerono"""
    def __init__(self):
        self.scale = 2
        self.omega = 0.7
        self.period = 2 * np.pi / self.omega

    def x(self, t):
        return self.scale * np.cos(self.omega * t)

    def y(self, t):
        return self.scale * np.sin(2 * self.omega * t) / 2

    def z(self, t):
        return self.scale * np.zeros((len(t)))

    def dotx(self, t):
        return -self.scale * self.omega * np.sin(self.omega * t)

    def doty(self, t):
        return self.scale * self.omega * np.cos(2 * self.omega * t)

    def dotz(self, t):
        try:
            return self.scale * np.zeros((len(t)))
        except:
            return 0

    def theta(self, t):
        return self.omega * t

    def omegaBody(self, t):
        """only works with one t"""
        return self.omega

    def R_WB(self, t):
        """only works with one t"""
        theta = np.arctan2(self.doty(t), self.dotx(t))
        ct = np.cos(theta)
        st = np.sin(theta)
        return np.array([[ct, -st, 0], [st, ct, 0], [0, 0, 1]])

    def velocityBody(self, t):
        """only works with one t"""
        R = self.R_WB(t)
        vW = np.array([[self.dotx(t)], [self.doty(t)], [self.dotz(t)]])
        vB = np.matmul(R.T, vW)
        assert (abs(vB[1]) < 1e-6 and abs(vB[2]) < 1e-6)
        return vB


def main():
    fig = plt.figure()
    plot3d = True
    if plot3d:
        ax = fig.add_subplot(projection='3d')
    else:
        ax = fig.add_subplot()

    # create the parametric curve
    curve = FigureInfinity()
    steps = 100
    t = np.arange(0, curve.period, curve.period / steps)
    x = curve.x(t)
    y = curve.y(t)
    z = curve.z(t)

    if plot3d:
        point, = ax.plot([x[0]], [y[0]], [z[0]], 'o')
        line, = ax.plot(x, y, z, label='parametric curve')
        ax.legend()
        ax.set_zlim([-1.5 * curve.scale, 1.5 * curve.scale])
        ax.set_zlabel('z (m)')
    else:
        point, = ax.plot([x[0]], [y[0]], 'o')
        line, = ax.plot(x, y, label='parametric curve')
        ax.legend()

    ax.set_xlim([-1.5 * curve.scale, 1.5 * curve.scale])
    ax.set_ylim([-1.5 * curve.scale, 1.5 * curve.scale])
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid()

    # second option - move the point position at every frame
    def update_point(n, x, y, z, point):
        point.set_data(np.array([x[n], y[n]]))
        if plot3d:
            point.set_3d_properties(z[n], 'z')
        vB = curve.velocityBody(t[n])
        print('step {}:{}, angular rate body {:.3f}, velocity body {:.3f}.'.format(
            n, steps, curve.omegaBody(t[n]), vB[0, 0]))
        return point

    ani=animation.FuncAnimation(fig, update_point, 99, fargs=(x, y, z, point))
    plt.show()


if __name__ == '__main__':
    main()
