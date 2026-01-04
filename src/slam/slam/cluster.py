import numpy as np

from geometry_msgs.msg import Point32

class Cluster:
    def __init__(self, pt):
        self.cluster = []
        self.cluster.append({'x':pt.x, 'y':pt.y})

    def centroid(self, off_x=0.0, off_y=0.0):
        x = sum(p['x'] + off_x for p in self.cluster)/len(self.cluster)
        y = sum(p['y'] + off_y for p in self.cluster)/len(self.cluster)

        return np.array([x, y])

    def length(self):
        return len(self.cluster)

    def add_point(self, pt):
        self.cluster.append({'x': pt.x, 'y': pt.y})

    def covariance(self, off_x=0.0, off_y=0.0):
        xs = [p['x'] + off_x for p in self.cluster]
        ys = [p['y'] + off_y for p in self.cluster]
        cov = np.cov([xs, ys])
        return cov
    
    def points(self, x=0.0, y=0.0):
        return [
            {'x': p['x'] + x, 'y': p['y'] + y}
            for p in self.cluster
        ]