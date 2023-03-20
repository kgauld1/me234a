import random
import numpy as np
from perlin_noise import PerlinNoise
import matplotlib.pyplot as plt

WALL = 0
UNKNOWN = 1
ONDECK = 2
PROCESSED = 3

def generate_world(M, N, wall_prob = 0.1, adj_prob = 0.4):
    state = np.ones((M, N)) * UNKNOWN

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

    # h = showgrid(state)
    start = [random.randint(1,M-1), random.randint(1,N-1)]
    while state[start[0], start[1]] == WALL:
        start = [random.randint(1,M-1), random.randint(1,N-1)]
    end = [random.randint(1,M-1), random.randint(1,N-1)]
    while state[end[0], end[1]] == WALL:
        end = [random.randint(1,M-1), random.randint(1,N-1)]
    return state, start, end


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

    # h = showgrid(state)
    start = [random.randint(1,M-1), random.randint(1,N-1)]
    while state[start[0], start[1]] == WALL:
        start = [random.randint(1,M-1), random.randint(1,N-1)]
    end = [random.randint(1,M-1), random.randint(1,N-1)]
    while state[end[0], end[1]] == WALL:
        end = [random.randint(1,M-1), random.randint(1,N-1)]

    mask = maskgen(M,N)
    return state, costmap, mask, start, end

def maskgen(M,N):
    noise = PerlinNoise(octaves=10, seed=1)
    xpix, ypix = M, N
    pic = np.array([[noise([i/xpix, j/ypix]) for j in range(xpix)] for i in range(ypix)])
    pic -= np.min(pic)
    pic /= np.ptp(pic)
    pic = np.around(np.array(pic)-0.1,0)
    return pic

