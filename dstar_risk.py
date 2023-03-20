from utils import *
from d_star_lite import d_star

def replan_path(dstarobj, state, replan_ctr=0):
    path = dstarobj.get_path()
    if path == None: 
        return path, replan_ctr
    for p in path:
        if state[p] == WALL:
            #print("REPLANNING!")
            dstarobj.replan([p])
            return replan_path(dstarobj, state, replan_ctr=replan_ctr+1)
    return path, replan_ctr

def run_all_planners(N,M):
    state, costmap, mask, start, goal = generate_world(N,M)
    maskstate = state.copy()
    maskstate[np.where(mask==1)] = RISKY
    costmap = np.ones((N,M))

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

    return state, maskstate, costmap, norisk_path, \
        opp_path, opp_ctr, av_path, av_ctr, med_path, med_ctr

if __name__ == "__main__":
    state, maskstate, costmap, norisk_path, opp_path, opp_ctr, \
        av_path, av_ctr, med_path, med_ctr  = run_all_planners(50,100)
    
    _, axs = plt.subplots(4, 1, figsize=(6, 9))
    axs = axs.flatten()

    if norisk_path != None:
        print("DSTAR PATH LEN:", len(norisk_path))
        show_path(norisk_path, state, axs[0])
    else:
        print("NO BASE DSTAR PATH FOUND. EXITING.")
        quit()

    if opp_path != None:
        print("OPPORTUNISTIC PATH LEN:", len(opp_path), 
            "\tPROPORTION:", len(opp_path)/len(norisk_path),
            "\tREPLANS:", opp_ctr)
        show_path(opp_path, maskstate, axs[1])
    else:
        print("NO OPPORTUNISTIC PATH FOUND")
        print("REPLANNED", opp_ctr, "TIMES")
        show_path([], np.zeros(maskstate.shape), axs[1])
    
    if av_path != None:
        print("RISK-AVERSE PATH LEN:", len(av_path), 
            "\tPROPORTION:", len(av_path)/len(norisk_path),
            "\tREPLANS:", av_ctr)
        show_path(av_path, maskstate, axs[2])
    else:
        print("NO RISK-AVERSE PATH FOUND")
        print("REPLANNED", av_ctr, "TIMES")
        show_path([], np.zeros(maskstate.shape), axs[2])
    
    if med_path != None:
        print("MODERATE PATH LEN:", len(med_path), 
            "\tPROPORTION:", len(med_path)/len(norisk_path),
            "\tREPLANS:", med_ctr)
        show_path(med_path, maskstate, axs[3])
    else:
        print("NO MODERATE PATH FOUND")
        print("REPLANNED",med_ctr,"TIMES")
        show_path([], np.zeros(maskstate.shape), axs[3])

    plt.show()