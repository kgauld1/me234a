import heapq

class Dstar:
    def __init__(self, grid):
        self.grid = grid
        self.start = None
        self.goal = None
        self.open_set = []
        self.back_pointers = {}
        self.g = {}
        self.rhs = {}
        self.km = 0
    
    def initialize(self, start, goal):
        self.start = start
        self.goal = goal
        self.open_set = [(self.calculate_key(start), start)]
        self.back_pointers = {}
        self.g = {cell: float('inf') for cell in self.grid}
        self.rhs = {cell: float('inf') for cell in self.grid}
        self.rhs[self.goal] = 0
        self.g[self.goal] = 0
    
    def calculate_key(self, cell):
        return (min(self.g[cell], self.rhs[cell]) + self.heuristic(cell, self.start) + self.km, min(self.g[cell], self.rhs[cell]))
    
    def heuristic(self, cell1, cell2):
        return abs(cell1[0] - cell2[0]) + abs(cell1[1] - cell2[1])
    
    def update_vertex(self, cell):
        if cell != self.start:
            self.rhs[cell] = min(self.grid.neighbours(cell, allow_diagonal=True), key=lambda x: self.g[x] + self.grid.cost(cell, x, self.g)).cost + self.g[cell]
        if cell in self.open_set:
            self.open_set.remove(cell)
        if self.g[cell] != self.rhs[cell]:
            heapq.heappush(self.open_set, (self.calculate_key(cell), cell))
    
    def compute_shortest_path(self):
        while self.open_set and (self.calculate_key(self.start) < self.open_set[0][0] or self.rhs[self.start] != self.g[self.start]):
            k_old = self.open_set[0][0]
            cell = heapq.heappop(self.open_set)[1]
            if k_old < self.calculate_key(cell):
                heapq.heappush(self.open_set, (self.calculate_key(cell), cell))
            elif self.g[cell] > self.rhs[cell]:
                self.g[cell] = self.rhs[cell]
                for neighbour in self.grid.neighbours(cell, allow_diagonal=True):
                    self.update_vertex(neighbour)
            else:
                self.g[cell] = float('inf')
                for neighbour in self.grid.neighbours(cell, allow_diagonal=True):
                    self.update_vertex(neighbour)
                self.update_vertex(cell)
    
    def plan(self, start, goal):
        self.initialize(start, goal)
        self.compute_shortest_path()
        while self.start != self.goal:
            next_cell = min(self.grid.neighbours(self.start, allow_diagonal=True), key=lambda x: self.grid.cost(self.start, x, self.g) + self.g[x])
            self.back_pointers[next_cell] = self.start
            self.start = next_cell
            self.update_vertex(self.start)
            self.compute_shortest_path()
        path = [self.goal]
        while path[-1] != self.start:
            path.append(self.back_pointers[path[-1]])
        return path[::-1]
