from utils import *
from astar_risk import astar, astar_riskaware
from dstar_risk import replan_path
from d_star_lite import d_star

def run_all_dstar(state, costmap, maskstate, mask, start, goal):
    dstartime = time.perf_counter()
    dstar_norisk = d_star(state.copy(), costmap.copy(), start, goal)
    norisk_path = dstar_norisk.get_path()
    dstartime = time.perf_counter()-dstartime

    opptime = time.perf_counter()
    dstar_opp = d_star(maskstate.copy(), costmap.copy(), start, goal)
    opp_path, opp_ctr = replan_path(dstar_opp, state)
    opptime = time.perf_counter()-opptime

    costmap2 = costmap.copy()
    costmap2[np.where(mask==1)] = 10**6
    avtime = time.perf_counter()
    dstar_av = d_star(maskstate.copy(), costmap2, start, goal)
    av_path, av_ctr = replan_path(dstar_av, state)
    avtime = time.perf_counter()-avtime

    costmap3 = costmap.copy()
    costmap3[np.where(mask==1)] = 10
    medtime = time.perf_counter()
    dstar_med = d_star(maskstate.copy(), costmap3, start, goal)
    med_path, med_ctr = replan_path(dstar_med, state)
    medtime = time.perf_counter()-medtime

    return norisk_path, dstartime, opp_path, opp_ctr, opptime, av_path, av_ctr, avtime, med_path, med_ctr, medtime

def run_all_astar(state, costmap, maskstate, mask, start, goal):
    c_path_1 = lambda x: 1*x[1] + 1*m_dist(x[0], goal) + 1*m_dist(x[0], start)+ 0*mask[x[0]]
    astartime = time.perf_counter()
    astar_path, finstate = astar(start, goal, 
                                 state.copy(), 
                                 costmap.copy(), 
                                 c_path_1)
    astartime = time.perf_counter()-astartime
    
    opptime = time.perf_counter()
    opp_path, opp_ctr = astar_riskaware(start, goal, 
                                        state.copy(), 
                                        maskstate.copy(), 
                                        costmap.copy(), 
                                        c_path_1)
    opptime = time.perf_counter()-opptime
    
    c_path_2 = lambda x: 1*x[1] + 1*m_dist(x[0], goal) + 1*m_dist(x[0], start) + 10**6*mask[x[0]]
    avtime = time.perf_counter()
    av_path, av_ctr = astar_riskaware(start, goal, 
                                      state.copy(), 
                                      maskstate.copy(), 
                                      costmap.copy(), c_path_2)
    avtime = time.perf_counter()-avtime
    
    c_path_3 = lambda x: 1*x[1] + 1*m_dist(x[0], goal) + 1*m_dist(x[0], start) + 10*mask[x[0]]
    medtime = time.perf_counter()
    med_path, med_ctr = astar_riskaware(start, goal, 
                                        state.copy(), 
                                        maskstate.copy(), 
                                        costmap.copy(), c_path_3)
    medtime = time.perf_counter()-medtime

    return astar_path, astartime, opp_path, opp_ctr, opptime, av_path, av_ctr, avtime, med_path, med_ctr, medtime

def run_all_planners(N,M,risk_t=-0.1,uniform=True):
    state, costmap, mask, start, goal = generate_world(N,M,risk_t=risk_t)
    maskstate = state.copy()
    maskstate[np.where(mask==1)] = RISKY
    if uniform: costmap = np.ones((N,M))

    norisk_dstar, norisk_dstar_t, \
        opp_path_dstar, opp_ctr_dstar, opp_dstar_t, \
        av_path_dstar, av_ctr_dstar, av_dstar_t, \
        med_path_dstar, med_ctr_dstar, med_dstar_t = run_all_dstar(state, costmap, maskstate, mask, start, goal)
    norisk_astar, norisk_astar_t, \
        opp_path_astar, opp_ctr_astar, opp_astar_t,\
        av_path_astar, av_ctr_astar, av_astar_t, \
        med_path_astar, med_ctr_astar, med_astar_t = run_all_astar(state, costmap, maskstate, mask, start, goal)
    
    return state, costmap, mask, maskstate, \
        norisk_dstar, norisk_dstar_t, opp_path_dstar, opp_ctr_dstar, opp_dstar_t,\
        av_path_dstar, av_ctr_dstar, av_dstar_t, med_path_dstar, med_ctr_dstar, med_dstar_t,\
        norisk_astar, norisk_astar_t, opp_path_astar, opp_ctr_astar, opp_astar_t,\
        av_path_astar, av_ctr_astar, av_astar_t, med_path_astar, med_ctr_astar, med_astar_t

if __name__ == "__main__":
    state, costmap, mask, maskstate, \
        norisk_dstar, norisk_dstar_t, opp_path_dstar, opp_ctr_dstar, opp_dstar_t,\
        av_path_dstar, av_ctr_dstar, av_dstar_t, med_path_dstar, med_ctr_dstar, med_dstar_t,\
        norisk_astar, norisk_astar_t, opp_path_astar, opp_ctr_astar, opp_astar_t,\
        av_path_astar, av_ctr_astar, av_astar_t, \
        med_path_astar, med_ctr_astar, med_astar_t = run_all_planners(50,50,uniform=False)

    _, axs = plt.subplots(2, 5, figsize=(9, 5))
    axs = axs.flatten()

    # for ax in axs:
    #     ax.xaxis.set_tick_params(labelbottom=False)
    #     ax.yaxis.set_tick_params(labelleft=False)
    axs[0].set_title("True Map")
    axs[1].set_title("Base D*")
    axs[2].set_title("Opportunistic D*")
    axs[3].set_title("Risk-Averse D*")
    axs[4].set_title("Moderately Averse D*")

    axs[5].set_title("Risk Map")
    axs[6].set_title("Base A*")
    axs[7].set_title("Opportunistic A*")
    axs[8].set_title("Risk-Averse A*")
    axs[9].set_title("Moderately Averse A*")
    
    axs[0].imshow(state)
    axs[5].imshow(maskstate)

    if norisk_dstar != None:
        print("D* PATH LEN:", len(norisk_dstar))
        show_path(norisk_dstar, state, axs[1])
    else:
        print("NO BASE D* PATH FOUND. EXITING.")
        quit()
    print("TOOK", norisk_dstar_t, "SECONDS")

    if opp_path_dstar != None:
        print("OPPORTUNISTIC D* PATH LEN:", len(opp_path_dstar), 
            "\tPROPORTION:", len(opp_path_dstar)/len(norisk_dstar),
            "\tREPLANS:", opp_ctr_dstar)
        show_path(opp_path_dstar, maskstate, axs[2])
    else:
        print("NO OPPORTUNISTIC D* PATH FOUND")
        print("REPLANNED", opp_ctr_dstar, "TIMES")
        show_path([], np.zeros(maskstate.shape), axs[2])
    print("TOOK", opp_dstar_t, "SECONDS")

    if av_path_dstar != None:
        print("RISK-AVERSE D* PATH LEN:", len(av_path_dstar), 
            "\tPROPORTION:", len(av_path_dstar)/len(norisk_dstar),
            "\tREPLANS:", av_ctr_dstar)
        show_path(av_path_dstar, maskstate, axs[3])
    else:
        print("NO RISK-AVERSE D* PATH FOUND")
        print("REPLANNED", av_ctr_dstar, "TIMES")
        show_path([], np.zeros(maskstate.shape), axs[3])
    print("TOOK", av_dstar_t, "SECONDS")
    
    if med_path_dstar != None:
        print("MODERATE D* PATH LEN:", len(med_path_dstar), 
            "\tPROPORTION:", len(med_path_dstar)/len(norisk_dstar),
            "\tREPLANS:", med_ctr_dstar)
        show_path(med_path_dstar, maskstate, axs[4])
    else:
        print("NO MODERATE D* PATH FOUND")
        print("REPLANNED",med_ctr_dstar,"TIMES")
        show_path([], np.zeros(maskstate.shape), axs[4])
    print("TOOK", med_dstar_t, "SECONDS")
    

    if norisk_astar != None:
        print("A* PATH LEN:", len(norisk_astar))
        show_path(norisk_astar, state, axs[6])
    else:
        print("NO BASE A* PATH FOUND. EXITING.")
        quit()
    print("TOOK", norisk_astar_t, "SECONDS")

    if opp_path_astar != None:
        print("OPPORTUNISTIC A* PATH LEN:", len(opp_path_astar), 
            "\tPROPORTION:", len(opp_path_astar)/len(norisk_astar),
            "\tREPLANS:", opp_ctr_astar)
        show_path(opp_path_astar, maskstate, axs[7])
    else:
        print("NO OPPORTUNISTIC A* PATH FOUND")
        print("REPLANNED", opp_ctr_astar, "TIMES")
        show_path([], np.zeros(maskstate.shape), axs[7])
    print("TOOK", opp_astar_t, "SECONDS")
    
    if av_path_astar != None:
        print("RISK-AVERSE A* PATH LEN:", len(av_path_astar), 
            "\tPROPORTION:", len(av_path_astar)/len(norisk_astar),
            "\tREPLANS:", av_ctr_astar)
        show_path(av_path_astar, maskstate, axs[8])
    else:
        print("NO RISK-AVERSE PATH FOUND")
        print("REPLANNED", av_ctr_astar, "TIMES")
        show_path([], np.zeros(maskstate.shape), axs[8])
    print("TOOK", av_astar_t, "SECONDS")
    
    if med_path_astar != None:
        print("MODERATE A* PATH LEN:", len(med_path_astar), 
            "\tPROPORTION:", len(med_path_astar)/len(norisk_astar),
            "\tREPLANS:", med_ctr_astar)
        show_path(med_path_astar, maskstate, axs[9])
    else:
        print("NO MODERATE PATH FOUND")
        print("REPLANNED",med_ctr_astar,"TIMES")
        show_path([], np.zeros(maskstate.shape), axs[9])
    print("TOOK", med_astar_t, "SECONDS")

    plt.show()