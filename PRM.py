from sklearn.neighbors import NearestNeighbors
import numpy as np

class PRM:
    def __init__(self, environment, num_samples, connection_radius, start, goal):
        self.environment = environment
        self.num_samples = num_samples
        self.connection_radius = connection_radius
        self.start = start
        self.goal = goal
        self.samples = [start, goal]
        self.roadmap = {}
        self.roadmap[start] = []
        self.roadmap[goal] = []
    
    # Randomly select samples in the environment
    def generate_samples(self):
        for i in range(self.num_samples):
            x = np.random.rand()*self.environment.size_x
            y = np.random.rand()*self.environment.size_y
            if not self.environment.check_collision(x,y):
                self.roadmap[(x,y)] = []
                self.samples.append((x,y))
                
                # Uncomment line below to display all sampled points
                # pl.plot(x,y,"black", marker = "x", markersize = 3)
    

    # Determines whether a line intersects obstacles
    def collision_free_path(self, line_start, line_end):
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        
        def intersect(A, B, C, D):
            return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

        for tri in self.environment.obs:
            triangle_vertices =[(tri.x0, tri.y0), (tri.x1, tri.y1), (tri.x2, tri.y2)]
            for i in range(3):
                j = (i+1)%3
                if intersect(line_start, line_end, triangle_vertices[i], triangle_vertices[j]):
                    return(False)
        return(True)

    # Joins samples to its neighbours to form a roadmap
    def build_roadmap(self):
        neigh = NearestNeighbors(radius=self.connection_radius, algorithm = 'auto').fit(self.samples)
        for i, (x,y) in enumerate(self.samples):
            neighbours = neigh.radius_neighbors([[x,y]])
            for k in range(len(np.asarray(neighbours[1][0]))):
                line_end = self.samples[np.asarray(neighbours[1][0][k])]
                if self.collision_free_path((x,y), line_end):
                    self.roadmap[(x,y)].append((line_end, np.asarray(neighbours[0][0][k])))
                    
                    # Uncomment line below to display roadmap
                    # pl.plot([x, line_end[0]], [y, line_end[1]], "green", linewidth =0.1)