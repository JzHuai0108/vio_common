#!/usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting
from matplotlib import animation


class FigureInfinity():
    """figure lemniscate of Gerono"""
    def __init__(self, omega = 1.0, scale = 2.0):
        self.scale = scale
        self.omega = omega
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
        """only works with one t
        return omega z
        """
        theta = self.omega * t
        ct = np.cos(theta)
        st = np.sin(theta)
        s2t = np.sin(2 * theta)
        return self.omega * (ct + st * s2t) / (st * st - s2t * s2t + 1)

    def tangentialAngle(self, t):
        return np.arctan2(self.doty(t), self.dotx(t))

    def R_WB(self, t):
        """only works with one t"""
        theta = np.arctan2(self.doty(t), self.dotx(t))
        ct = np.cos(theta)
        st = np.sin(theta)
        return np.array([[ct, -st, 0], [st, ct, 0], [0, 0, 1]])

    def velocityBody(self, t):
        """only works with one t
        return vx, since vy and vz are zero
        """
        R = self.R_WB(t)
        vW = np.array([[self.dotx(t)], [self.doty(t)], [self.dotz(t)]])
        vB = np.matmul(R.T, vW)
        assert (abs(vB[1]) < 1e-6 and abs(vB[2]) < 1e-6)
        return vB[0, 0]


class Circle(object):
    def __init__(self, velocitybody=2.0, radius=2.0):
        self.radius = radius
        self.omega = velocitybody / radius
        self.period = 2 * np.pi / self.omega
        self.scale = self.radius

    def velocityBody(self, t):
        return self.omega * self.radius

    def omegaBody(self, t):
        return self.omega

    def x(self, t):
        theta = self.omega * t
        return self.radius * np.cos(theta)

    def y(self, t):
        theta = self.omega * t
        return self.radius * np.sin(theta)

    def z(self, t):
        return np.zeros((len(t)))


class LineSegment(object):
    def __init__(self, omega = 1.0, scale = 1.0):
        self.omega = omega
        self.scale = scale
        self.period = np.pi * 2 / omega

    def x(self, t):
        return self.scale * (t - np.sin(self.omega * t) / self.omega - self.period * 0.5)

    def y(self, t):
        return np.zeros((len(t)))

    def z(self, t):
        return np.zeros((len(t)))

    def velocityBody(self, t):
        return self.scale * (1 - np.cos(self.omega * t))

    def accelBody(self, t):
        return self.scale * self.omega * np.sin(self.omega * t)

    def omegaBody(self, t):
        return 0


def main():
    fig = plt.figure()
    plot3d = True
    if plot3d:
        ax = fig.add_subplot(projection='3d')
    else:
        ax = fig.add_subplot()

    # create the parametric curve
    curve = LineSegment(1.3, 1.5)
    # curve = FigureInfinity(0.7)
    # curve = Circle(0.7)
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
        angularRate = curve.omegaBody(t[n])
        vx = curve.velocityBody(t[n])

        print('step {}:{}, velocity body {:.3f}, angular rate body {:.3f}.'.format(n, steps, vx, angularRate))
        return point

    ani=animation.FuncAnimation(fig, update_point, steps, fargs=(x, y, z, point))
    plt.show()


if __name__ == '__main__':
    main()
