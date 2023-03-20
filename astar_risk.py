import numpy as np
import matplotlib.pyplot as plt
from utils import *

def astar(start, goal, s, c, c_path):
    state = s.copy()
    costmap = c.copy()
    ondeck = [(start, 0)]
    prior_node = {start:None}

    while ondeck:
        ondeck.sort(key=c_path)
        current, cur_cost = ondeck.pop(0)
        
        if current == goal:
            state[current] = PROCESSED
            return (build_path(goal, prior_node), state)
        
        for d in [(0,1),(0,-1),(1,0),(-1,0)]:
            nnode = (current[0] + d[0], current[1] + d[1])
            if 0<=nnode[0]<len(state) and 0<=nnode[1]<len(state[0]):
                if state[nnode] == UNKNOWN:
                    state[nnode] = ONDECK
                    ondeck.append((nnode, cur_cost + costmap[nnode]))
                    prior_node[nnode] = current
        
        state[current] = PROCESSED
    
    return (None, state)

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
                    # plt.imshow(rs)
                    # plt.show()
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

def run_all_planners(N,M):
    state, costmap, mask, start, goal = generate_world(N,M)
    maskstate = state.copy()
    maskstate[np.where(mask==1)] = RISKY
    costmap = np.ones((N,M))

    c_path_1 = lambda x: 1*x[1] + 1*m_dist(x[0], goal) + 1*m_dist(x[0], start)+ 0*mask[x[0]]
    astar_path, finstate = astar(start, goal, 
                                 state.copy(), 
                                 costmap.copy(), 
                                 c_path_1)
    
    opp_path, opp_ctr = astar_riskaware(start, goal, 
                                        state.copy(), 
                                        maskstate.copy(), 
                                        costmap.copy(), 
                                        c_path_1)
    
    c_path_2 = lambda x: 1*x[1] + 1*m_dist(x[0], goal) + 1*m_dist(x[0], start) + 10**6*mask[x[0]]
    av_path, av_ctr = astar_riskaware(start, goal, 
                                      state.copy(), 
                                      maskstate.copy(), 
                                      costmap.copy(), c_path_2)
    
    c_path_3 = lambda x: 1*x[1] + 1*m_dist(x[0], goal) + 1*m_dist(x[0], start) + 10*mask[x[0]]
    med_path, med_ctr = astar_riskaware(start, goal, 
                                        state.copy(), 
                                        maskstate.copy(), 
                                        costmap.copy(), c_path_3)

    return state, maskstate, costmap, astar_path, \
        opp_path, opp_ctr, av_path, av_ctr, med_path, med_ctr



def show_path(path, state, ax):
    drawstate = state.copy()
    for p in path:
        drawstate[p] = float('inf')
    ax.imshow(drawstate)
    #plt.show()

if __name__ == "__main__":
    state, maskstate, costmap, astar_path, \
        opp_path, opp_ctr, av_path, av_ctr, med_path, med_ctr = run_all_planners(50,100)
    
    print("ASTAR PATH LEN:", len(astar_path))
    print("OPPORTUNISTIC PATH LEN:", len(opp_path), 
          "\tPROPORTION:", len(opp_path)/len(astar_path),
          "\tREPLANS:", opp_ctr)
    print("RISK-AVERSE PATH LEN:", len(av_path), 
          "\tPROPORTION:", len(av_path)/len(astar_path),
          "\tREPLANS:", av_ctr)
    print("MODERATE PATH LEN:", len(med_path), 
          "\tPROPORTION:", len(med_path)/len(astar_path),
          "\tREPLANS:", opp_ctr)
    
    _, axs = plt.subplots(4, 1, figsize=(6, 9))
    axs = axs.flatten()

    show_path(astar_path, state, axs[0])
    show_path(opp_path, maskstate, axs[1])
    show_path(av_path, maskstate, axs[2])
    show_path(med_path, maskstate, axs[3])

    plt.show()