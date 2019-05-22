from SamplingRegion import RectangularSamplingRegion, CircularSamplingRegion
import matplotlib.pyplot as plt

rectangle = RectangularSamplingRegion(0, 5, 0, 10)
circle = CircularSamplingRegion(9, 5, 2)

rectangle_samples = [rectangle.random_configuration().configuration for _ in range(10 ** 2)]
circle_samples = [circle.random_configuration().configuration for _ in range(10 ** 2)]
rx, ry, ra = zip(*rectangle_samples)
cx, cy, ca = zip(*circle_samples)

circle.draw('green')

plt.scatter(rx, ry, color='red', s=2)
plt.scatter(cx, cy, color='blue', s=2)
plt.show()
