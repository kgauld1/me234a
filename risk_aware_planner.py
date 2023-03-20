from utils import *
from astar_risk import astar, astar_riskaware
from dstar_risk import replan_path
from d_star_lite import d_star

def run_all_dstar(state, costmap, maskstate, mask, start, goal):
    dstar_norisk = d_star(state.copy(), costmap.copy(), start, goal)
    norisk_path = dstar_norisk.get_path()

    dstar_opp = d_star(maskstate.copy(), costmap.copy(), start, goal)
    opp_path, opp_ctr = replan_path(dstar_opp, state)

    costmap2 = costmap.copy()
    costmap2[np.where(mask==1)] = 10**6
    dstar_av = d_star(maskstate.copy(), costmap2, start, goal)
    av_path, av_ctr = replan_path(dstar_av, state)

    costmap3 = costmap.copy()
    costmap3[np.where(mask==1)] = 10
    dstar_med = d_star(maskstate.copy(), costmap3, start, goal)
    med_path, med_ctr = replan_path(dstar_med, state)

    return norisk_path, opp_path, opp_ctr, av_path, av_ctr, med_path, med_ctr

def run_all_astar(state, costmap, maskstate, mask, start, goal):
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

    return astar_path, opp_path, opp_ctr, av_path, av_ctr, med_path, med_ctr

def run_all_planners(N,M):
    state, costmap, mask, start, goal = generate_world(N,M)
    maskstate = state.copy()
    maskstate[np.where(mask==1)] = RISKY
    costmap = np.ones((N,M))

    norisk_dstar, opp_path_dstar, opp_ctr_dstar, av_path_dstar, av_ctr_dstar, med_path_dstar, med_ctr_dstar = run_all_dstar(state, costmap, maskstate, mask, start, goal)
    norisk_astar, opp_path_astar, opp_ctr_astar, av_path_astar, av_ctr_astar, med_path_astar, med_ctr_astar = run_all_astar(state, costmap, maskstate, mask, start, goal)
    
    return state, costmap, mask, maskstate, \
        norisk_dstar, opp_path_dstar, opp_ctr_dstar, \
        av_path_dstar, av_ctr_dstar, med_path_dstar, med_ctr_dstar, \
        norisk_astar, opp_path_astar, opp_ctr_astar, \
        av_path_astar, av_ctr_astar, med_path_astar, med_ctr_astar

if __name__ == "__main__":
    state, costmap, mask, maskstate, \
        norisk_dstar, opp_path_dstar, opp_ctr_dstar, \
        av_path_dstar, av_ctr_dstar, med_path_dstar, med_ctr_dstar, \
        norisk_astar, opp_path_astar, opp_ctr_astar, av_path_astar, av_ctr_astar, \
        med_path_astar, med_ctr_astar = run_all_planners(50,100)

    _, axs = plt.subplots(4, 2, figsize=(6, 9))
    axs = axs.flatten()

    if norisk_dstar != None:
        print("D* PATH LEN:", len(norisk_dstar))
        show_path(norisk_dstar, state, axs[0])
    else:
        print("NO BASE D* PATH FOUND. EXITING.")
        quit()

    if opp_path_dstar != None:
        print("OPPORTUNISTIC D* PATH LEN:", len(opp_path_dstar), 
            "\tPROPORTION:", len(opp_path_dstar)/len(norisk_dstar),
            "\tREPLANS:", opp_ctr_dstar)
        show_path(opp_path_dstar, maskstate, axs[2])
    else:
        print("NO OPPORTUNISTIC D* PATH FOUND")
        print("REPLANNED", opp_ctr_dstar, "TIMES")
        show_path([], np.zeros(maskstate.shape), axs[2])
    
    if av_path_dstar != None:
        print("RISK-AVERSE D* PATH LEN:", len(av_path_dstar), 
            "\tPROPORTION:", len(av_path_dstar)/len(norisk_dstar),
            "\tREPLANS:", av_ctr_dstar)
        show_path(av_path_dstar, maskstate, axs[4])
    else:
        print("NO RISK-AVERSE D* PATH FOUND")
        print("REPLANNED", av_ctr_dstar, "TIMES")
        show_path([], np.zeros(maskstate.shape), axs[4])
    
    if med_path_dstar != None:
        print("MODERATE D* PATH LEN:", len(med_path_dstar), 
            "\tPROPORTION:", len(med_path_dstar)/len(norisk_dstar),
            "\tREPLANS:", med_ctr_dstar)
        show_path(med_path_dstar, maskstate, axs[6])
    else:
        print("NO MODERATE D* PATH FOUND")
        print("REPLANNED",med_ctr_dstar,"TIMES")
        show_path([], np.zeros(maskstate.shape), axs[6])
    

    if norisk_astar != None:
        print("A* PATH LEN:", len(norisk_astar))
        show_path(norisk_astar, state, axs[1])
    else:
        print("NO BASE A* PATH FOUND. EXITING.")
        quit()

    if opp_path_astar != None:
        print("OPPORTUNISTIC A* PATH LEN:", len(opp_path_astar), 
            "\tPROPORTION:", len(opp_path_astar)/len(norisk_astar),
            "\tREPLANS:", opp_ctr_astar)
        show_path(opp_path_astar, maskstate, axs[3])
    else:
        print("NO OPPORTUNISTIC A* PATH FOUND")
        print("REPLANNED", opp_ctr_astar, "TIMES")
        show_path([], np.zeros(maskstate.shape), axs[3])
    
    if av_path_astar != None:
        print("RISK-AVERSE A* PATH LEN:", len(av_path_astar), 
            "\tPROPORTION:", len(av_path_astar)/len(norisk_astar),
            "\tREPLANS:", av_ctr_astar)
        show_path(av_path_astar, maskstate, axs[5])
    else:
        print("NO RISK-AVERSE PATH FOUND")
        print("REPLANNED", av_ctr_astar, "TIMES")
        show_path([], np.zeros(maskstate.shape), axs[5])
    
    if med_path_astar != None:
        print("MODERATE A* PATH LEN:", len(med_path_astar), 
            "\tPROPORTION:", len(med_path_astar)/len(norisk_astar),
            "\tREPLANS:", med_ctr_astar)
        show_path(med_path_astar, maskstate, axs[7])
    else:
        print("NO MODERATE PATH FOUND")
        print("REPLANNED",med_ctr_astar,"TIMES")
        show_path([], np.zeros(maskstate.shape), axs[7])

    plt.show()