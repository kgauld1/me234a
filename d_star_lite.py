import numpy as np
import matplotlib.pyplot as plt
from math import inf

from priority_queue import PriorityQueue, Priority

WALL = 0
UNKNOWN = 1

class d_star:
    def __init__(self, state, costmap, start, end):
        self.M = len(state)
        self.N = len(state[0])
        self.state = state
        self.start = start
        self.end = end

        self.costmap = costmap

        self.U = PriorityQueue()
        self.rhs = np.ones((self.M, self.N)) * np.inf
        self.g = self.rhs.copy()
    
        self.rhs[end] = 0
        self.U.insert(end, Priority(self.heuristic(start, end), 0))

    def heuristic(self, u, v):
        return abs(u[0] - v[0]) + abs(u[1] - v[1])

    def is_wall(self, point):
        return self.state[point[0]][point[1]] == WALL

    def calculate_key(self, point):
        k1 = min(self.g[point], self.rhs[point]) + self.heuristic(self.start, point) + 0.001
        k2 = min(self.g[point], self.rhs[point])
        return Priority(k1, k2)

    def cost(self, u, v):
        if self.is_wall(u) or self.is_wall(v):
            return inf
        else:
            return self.costmap[v]

    def contain(self, u):
        return u in self.U.vertices_in_heap

    def update_vertex(self, u):
        if self.g[u] != self.rhs[u] and self.contain(u):
            # print("case 1")
            self.U.update(u, self.calculate_key(u))
        elif self.g[u] != self.rhs[u] and not self.contain(u):
            # print("case 2")
            self.U.insert(u, self.calculate_key(u))
        elif self.g[u] == self.rhs[u] and self.contain(u):
            # print("case 3")
            self.U.remove(u)

    def replan(self, points):
        for point in points:
            self.state[point[0]][point[1]] = WALL
            self.rhs[point] = inf
            self.rhs[self.start] = 200
            for n in self.get_neighbors(point):
                self.rhs[point] = inf
                self.update_vertex(n)

    def get_neighbors(self, u):
        n = []
        for dx, dy in [(-1, 0), (1, 0), (0, 1), (0, -1)]:
            x = u[0] + dx
            y = u[1] + dy
            if not self.is_wall(u) and 0<=x<self.M and 0<=y<self.N:
                n.append((x, y))
        return n

    def compute_shortest_path(self):
        while self.U.top_key() < self.calculate_key(self.start) or self.rhs[self.start] > self.g[self.start]:
            u = self.U.top()
            k_old = self.U.top_key()
            k_new = self.calculate_key(u)

            if k_old < k_new:
                # print("1")
                self.U.update(u, k_new)
            elif self.g[u] > self.rhs[u]:
                # print("2")
                self.g[u] = self.rhs[u]
                self.U.remove(u)
                for n in self.get_neighbors(u):
                    if n != self.end:
                        self.rhs[n] = min(self.rhs[n], self.cost(n, u) + self.g[u])
                    self.update_vertex(n)
            else:
                # print("3")
                g_old = self.g[u]
                self.g[u] = inf

                n_iter = self.get_neighbors(u)
                n_iter.append(u)
                for n in n_iter:
                    if self.rhs[n] == self.cost(n, u) + g_old:
                        if n != self.end:
                            min_s = inf
                            for n_s in self.get_neighbors(n):
                                temp = self.cost(n, n_s) + self.g[n_s]
                                if min_s > temp:
                                    min_s = temp
                            self.rhs[n] = min_s
                    self.update_vertex(u)

    def get_path(self, db = False):
        self.compute_shortest_path()

        if self.rhs[self.start] == inf:
            return None

        path = [self.start]
        worklist = [self.start]
        path = self.rec_path2(worklist, path, db)

        return path

    def rec_path2(self, worklist, path, db=False):
        node = worklist.pop(0)
        if db:
            print(node)
        if node == self.end:
            return path
        else:
            best_rhs = []
            best_nodes = []
            n_iter = self.get_neighbors(node)
            for n in n_iter:
                if n not in path:
                    best_rhs.append(self.rhs[n])
                    best_nodes.append(n)
            for i in range(len(best_rhs)):
                if best_rhs[i] == min(best_rhs):
                    path.append(best_nodes[i])
                    worklist.append(best_nodes[i])
                    val = self.rec_path2(worklist.copy(), path.copy(), db)
                    if not None:
                        return val
                    
