import numpy as np

from murl.world import World

WALL_OFFSET = 2.
CYLINDER_POSITION = np.array([.3, .2], dtype=np.float32)
CYLINDER_OFFSET = .3

SEGMENTS = list(map(lambda ps: (np.array(ps[0]), np.array(ps[1]), ps[2]),
                    [[[-WALL_OFFSET, -WALL_OFFSET], [0, 1], 2*WALL_OFFSET],
                     [[-WALL_OFFSET, WALL_OFFSET], [1, 0], 2*WALL_OFFSET],
                     [[WALL_OFFSET, WALL_OFFSET], [0, -1], 2*WALL_OFFSET],
                     [[WALL_OFFSET, -WALL_OFFSET], [-1, 0], 2*WALL_OFFSET]]))

class SimpleWorld(World):
    WALL_RADIUS = WALL_OFFSET - World.ROBOT_RADIUS
    CYLINDER_RADIUS = CYLINDER_OFFSET + World.ROBOT_RADIUS

    def __init__(self, rng):
        self._rng = rng

    def is_valid(self, poses):
        return ((-self.WALL_RADIUS < poses[...,0])
              & (poses[...,0] < self.WALL_RADIUS)
              & (-self.WALL_RADIUS < poses[...,1])
              & (poses[...,1] < self.WALL_RADIUS)
              & (np.sum((poses[...,:2] - CYLINDER_POSITION) ** 2, axis=-1) > self.CYLINDER_RADIUS ** 2))

    def get_dist(self, sources, angles):
        def dot(x, y):
            return x[...,0] * y[...,0] + x[...,1] * y[...,1]
        def cross(x, y):
            return x[...,0] * y[...,1] - x[...,1] * y[...,0]

        def intersect_segment(sources, angle, start, direction, length):
            Y = start - sources[:,:2]
            r = np.stack((np.cos(angle + sources[:,2]),
                          np.sin(angle + sources[:,2]))).T
            t = cross(Y, direction) / cross(r, direction)
            t1 = cross(Y, r) / cross(r, direction)

            return np.where((t >= 0) & (t1 >= 0) & (t1 <= length), t, float('inf'))

        def intersect_cylinder(sources, angle, center, radius):
            Y = center - sources[:,:2]
            d = np.stack((np.cos(angle + sources[:,2]),
                          np.sin(angle + sources[:,2]))).T
            b = dot(d, Y)
            c = dot(Y, Y) - radius**2
            disc = b**2 - dot(Y, Y) + radius**2

            mask = (disc > 0) & (b > 0) & (b ** 2 > disc)

            b[~mask] = float('inf')
            b[mask] -= np.sqrt(disc[mask])

            return b

        intersections = np.stack([np.stack(
                (intersect_segment(sources, angle, *SEGMENTS[0]),
                 intersect_segment(sources, angle, *SEGMENTS[1]),
                 intersect_segment(sources, angle, *SEGMENTS[2]),
                 intersect_segment(sources, angle, *SEGMENTS[3]),
                 intersect_cylinder(sources, angle, CYLINDER_POSITION, CYLINDER_OFFSET))
            ) for angle in angles])

        return np.amin(intersections, axis=(0,1))

    def sample(self, particles):
        particles[:,2] = self._rng.uniform(-np.pi, np.pi, size=len(particles))

        invalid = np.ones(shape=len(particles), dtype=np.bool)
        while np.any(invalid):
            count = np.ma.array(particles[:,0], mask=~invalid).count()
            particles[invalid,0] = self._rng.uniform(-self.WALL_RADIUS, self.WALL_RADIUS, size=count)
            particles[invalid,1] = self._rng.uniform(-self.WALL_RADIUS, self.WALL_RADIUS, size=count)
            invalid = ~self.is_valid(particles)
