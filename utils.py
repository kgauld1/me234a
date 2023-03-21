import random
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from perlin_noise import PerlinNoise
from math import inf
from priority_queue import PriorityQueue, Priority

RISKY = -1
WALL = 0
UNKNOWN = 1
ONDECK = 2
PROCESSED = 3

m_dist = lambda x1, x2: abs(x1[0]-x2[0]) + abs(x1[1]-x2[1])

def build_path(goal, pnodes):
    # Start the path at the end
    path = [goal]
    current = goal
    
    # While the current node has one before it, add the prior
    # node to the path then redo the check with this node
    while pnodes[current] != None:
        path.append(pnodes[current])
        current = pnodes[current]
    
    # Once the start node is reached, reverse the path (start -> goal) and return
    path.reverse()
    return path

def generate_world(M, N, wall_prob = 0.1, adj_prob = 0.4):
    state = np.ones((M, N)) * UNKNOWN
    costmap = -np.ones((M,N))


    state[ 0,0:] = WALL
    state[-1,0:] = WALL
    state[0:, 0] = WALL
    state[0:,-1] = WALL

    for i in range(1, M-1):
        for j in range(1, N-1):
            if random.randint(1,100) < wall_prob * 100:
                state[i, j] = WALL
            else:
                if state[i+1, j] == WALL or state[i-1, j] == WALL or state[i, j+1] == WALL or state[i, j-1] == WALL:
                    if random.randint(1,100) < adj_prob * 100:
                        state[i, j] = WALL
            if state[i,j] != WALL:
                costmap[i,j] = random.randint(1,10)

    start = (random.randint(1,M-1), random.randint(1,N-1))
    while state[start] == WALL:
        start = (random.randint(1,M-1), random.randint(1,N-1))
    end = (random.randint(1,M-1), random.randint(1,N-1))
    while state[end] == WALL:
        end = (random.randint(1,M-1), random.randint(1,N-1))

    mask = maskgen(M,N)
    return state, costmap, mask, start, end

def maskgen(M,N):
    noise = PerlinNoise(octaves=10, seed=1)
    xpix, ypix = N, M
    pic = np.array([[noise([i/xpix, j/ypix]) for j in range(xpix)] for i in range(ypix)])
    pic -= np.min(pic)
    pic /= np.ptp(pic)
    pic = np.around(np.array(pic)-0.1,0)
    return pic

def show_path(path, state, ax):
    drawstate = state.copy()
    for p in path:
        drawstate[p] = float('inf')
    ax.imshow(drawstate)
    #plt.show()