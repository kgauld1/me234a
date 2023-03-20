import random
import numpy as np
from perlin_noise import PerlinNoise
import matplotlib.pyplot as plt

RISKY = -1
WALL = 0
UNKNOWN = 1
ONDECK = 2
PROCESSED = 3

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

def astar(start, goal, s, c, c_path):
    state = s.copy()
    costmap = c.copy()
    # Tracks which nodes are in ondeck and when they were added to ondeck
    ondeck = [(start, 0)]
    # For any node, stores the node prior to it
    prior_node = {start:None}
    
    # While there is a node that has not been fully processed, process it
    while ondeck:
        # Sort ondeck by the path cost for each node, so the first entry
        # has the lowest path cost, then get the first entry 
        ondeck.sort(key=c_path)
        current, cur_cost = ondeck.pop(0)
        
        if current == goal:
            state[current] = PROCESSED
            # Return the path used to get to the goal and the state
            return (build_path(goal, prior_node), state)
        
        # Get the next nodes in each direction (up down left right)
        for i in (-1,1): # -1 for up/left, +1 for down/right
            for axis in (0, 1): # axis 0 for right/left, 1 for up/down
                
                # Find the next node in the specified direction
                nnode = (current[0] + (i if axis==1 else 0),
                         current[1] + (i if axis==0 else 0))
                
                # If the node has not been seen yet, add to ondeck and track
                # the node used to get to this one (the current node)
                if state[nnode] == UNKNOWN:
                    state[nnode] = ONDECK
                    ondeck.append((nnode, cur_cost + costmap[nnode]))
                    prior_node[nnode] = current
        
        # Mark the node as processed, check if we've reached the goal, then continue
        state[current] = PROCESSED
    
    # If a path could not be found
    return (None, state)

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


def astar_riskaware(start, goal, s, rs, cmap, c_path, replan_ctr=0):
    state = s.copy()
    riskystate = rs.copy()
    costmap = cmap.copy()
    ondeck = [(start, 0)]
    prior_node = {start:None}
    
    while ondeck:
        ondeck.sort(key=c_path)
        current, cur_cost = ondeck.pop(0)
        
        if current == goal:
            state[current] = PROCESSED
            path = build_path(goal, prior_node)
            if path is None:
                return None, ctr
            final_path = []

            for pidx in range(len(path)):
                p = path[pidx]
                if state[p] != WALL:
                    final_path.append(p)
                else:
                    print("REPLANNING!", p)
                    rs[p] = s[p]
                    rs[path[pidx-1]] = ONDECK
                    plt.imshow(rs)
                    plt.show()
                    epath, ctr = astar_riskaware(path[pidx-1], goal, s, rs, cmap, c_path, replan_ctr=replan_ctr+1)
                    if epath is None:
                        return None, ctr
                    final_path.extend(epath)
                    return final_path, ctr
            return build_path(goal, prior_node), replan_ctr

        for d in [(0,1),(0,-1),(1,0),(-1,0)]:
            nnode = (current[0] + d[0], current[1] + d[1])
            if 0<=nnode[0]<len(riskystate) and 0<=nnode[1]<len(riskystate[0]):
                if riskystate[nnode] == UNKNOWN or riskystate[nnode] == RISKY:
                    riskystate[nnode] = ONDECK
                    ondeck.append((nnode, cur_cost + costmap[nnode]))
                    prior_node[nnode] = current
        
        riskystate[current] = PROCESSED
    
    return None, replan_ctr
if __name__ == "__main__":
    state, costmap, mask, start, goal = generate_world(50,100)
    m_dist = lambda x1, x2: abs(x1[0]-x2[0]) + abs(x1[1]-x2[1])

    c_path_1 = lambda x: 1*x[1] + 1*m_dist(x[0], goal) + 1*m_dist(x[0], start)
    path, finstate = astar(start, goal, state.copy(), costmap, c_path_1)
    statepath = np.copy(state)
    for p in path:
        statepath[p] = -float('inf')
    
    plt.imshow(state)
    plt.show()
    plt.imshow(costmap)
    plt.show()
    plt.imshow(mask)
    plt.show()
    plt.imshow(statepath)
    plt.show()

    maskstate = state.copy()
    maskstate[np.where(mask==1)] = RISKY

    plt.imshow(maskstate)
    plt.show()

    c_path_2 = lambda x: 1*x[1] + 1*m_dist(x[0], goal) + 1*m_dist(x[0], start)# + 10**6*mask[x[0]]
    
    path, ctr = astar_riskaware(start, goal, state, maskstate, costmap, c_path_2)
    print("REPLANNED", ctr, "TIMES")
    maskstatepath = np.copy(maskstate)
    if path is None:
        print("NO PATH FOUND!")
        quit()
    
    for p in path:
        maskstatepath[p] = -float('inf')

    plt.imshow(maskstate)
    plt.show()
    plt.imshow(maskstatepath)
    plt.show()
