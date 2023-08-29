import pylab as pl
import heapq
import numpy as np
import time

class AStar:
    def __init__(self, prm, start, goal):
        self.prm = prm
        self.start = start
        self.goal = goal
        self.open_set = []
        self.came_from = {}
        self.g_score = {vertex: float('inf') for vertex in self.prm.roadmap.keys()}
        self.g_score[start] = 0
        self.f_score = {vertex: float('inf') for vertex in self.prm.roadmap.keys()}
        self.f_score[start] = self.distance(start, goal)
        heapq.heappush(self.open_set, (self.f_score[start], start))
    
    # Find distance between 2 points (used for heuristic cost estimation)
    def distance(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    
    # Backtracks and reconstructs the path from start to current
    def reconstruct_path(self, current):
        total_path = [current]
        while current in self.came_from:
            current = self.came_from[current]
            total_path.append(current)
        return(total_path[::-1])
    
    # Post-process path to find shortcut
    def shortcut_path(self, path, max_rep):
        # Remove unwanted points
        i = 0
        while i < len(path) - 2:
            if self.prm.collision_free_path(path[i], path[i+2]):
                path.pop(i+1)
            else:
                i += 1

        # Smooth out path edges
        for rep in range(max_rep):
            if len(path) > 2:
                i = np.random.randint(1, len(path) - 1)
            else:
                break
            t1 = np.random.uniform(0,0.2)
            t2 = np.random.uniform(0,0.2)

            q_t1 = (path[i][0] + t1 * (path[i + 1][0] - path[i][0]), 
                    path[i][1] + t1 * (path[i + 1][1] - path[i][1]))
            q_t2 = (path[i][0] - t2 * (path[i][0] - path[i-1][0]), 
                    path[i][1] - t2 * (path[i][1] - path[i-1][1]))
            
            if self.prm.collision_free_path(q_t1, q_t2):
                path.insert(i+1, q_t1)
                path[i] = q_t2
        return(path)

    # Main function
    def run(self):
        path =[]
        # A* Search for optimal path
        while self.open_set:
            n, current = heapq.heappop(self.open_set)

            # End search once goal is found
            if current == self.goal:
                path = self.reconstruct_path(current)
                break

            # Interates through neighbours to find lowest g-score path
            for neighbour, dist in self.prm.roadmap[current]:
                tentative_g_score = self.g_score[current] + dist
                if tentative_g_score < self.g_score[neighbour]:
                    self.came_from[neighbour] = current
                    self.g_score[neighbour] = tentative_g_score
                    self.f_score[neighbour] = tentative_g_score + self.distance(neighbour, self.goal)
                    heapq.heappush(self.open_set, (self.f_score[neighbour], neighbour))
        
        if path:
            # Display path before post-processing
            total_cost = self.g_score[path[-1]]
            print("Cost before post-processing =", total_cost, "No. of nodes =", len(path))
            pl.plot([point[0] for point in path], [point[1] for point in path], 'orange', linewidth=2)

            # Search for shortcut
            start = time.time()
            shortcut = self.shortcut_path(path, max_rep = 10)
            short_cost = 0
            for i in range(len(shortcut) - 1):
                short_cost += self.distance(shortcut[i], shortcut[i + 1])
            end=time.time()
            print("Cost after post-processing = ", short_cost, "No. of nodes =", len(shortcut))
            print("Time taken for post-processing =", end - start)
            
            # Display path and nodes after post-processing
            pl.plot([point[0] for point in shortcut], [point[1] for point in shortcut], 'green', linewidth=2)
            for point in shortcut:
                pl.plot(point[0], point[1], "green", marker = 'x', markersize = 5)
        
        else:
            # Display roadmap and all nodes if path not found
            for key, value in self.prm.roadmap.items():
                pl.plot(key[0], key[1], "black", marker = "x", markersize = 2)
                for point, dist in value:
                    pl.plot([key[0], point[0]], [key[1], point[1]], "green", linewidth =0.1)
            print("Path not found :(. Roadmap is displayed")