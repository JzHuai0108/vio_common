import point_to_parabola
import numpy as np


def testPointToParabola():
    import matplotlib.pyplot as plt
    coeffs = [0.5, -6, 10]

    x = np.arange(6-6, 6+6, 0.1)
    y = np.polyval(coeffs, x)

    point = np.random.rand(1, 2).flatten() * 5 + 5

    # Plotting the Graph
    plt.plot(x, y)
    plt.plot(point[0], point[1], 'r+')
    foot, dist = point_to_parabola.pointToParabola(coeffs, point)
    normal = np.array([[point[0], point[1]], [foot[0], foot[1]]])
    plt.plot(normal[:, 0], normal[:, 1])

    plt.title("Curve plotted using the given points")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')
    plt.grid(True)
    plt.show()
