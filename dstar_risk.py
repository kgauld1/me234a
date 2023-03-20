from utils import *
from d_star_lite import d_star


def replan_path(dstar, state):
    pass

def run_all_planners(N,M):
    state, costmap, mask, start, goal = generate_world(N,M)
    maskstate = state.copy()
    maskstate[np.where(mask==1)] = RISKY
    costmap = np.ones((N,M))

    dstar_norisk = d_star(state.copy(), costmap.copy(), start, goal)
    norisk_path = dstar_norisk.get_path()

    dstar_opp = d_star(maskstate.copy(), costmap.copy(), start, goal)
    opp_path = dstar_opp.get_path()

    costmap2 = costmap.copy()
    costmap2[np.where(mask==1)] = 10**6
    dstar_av = d_star(maskstate.copy(), costmap2, start, goal)
    av_path = dstar_av.get_path()

    costmap3 = costmap.copy()
    costmap3[np.where(mask==1)] = 10
    dstar_med = d_star(maskstate.copy(), costmap3, start, goal)
    med_path = dstar_med.get_path()

    return state, maskstate, costmap, norisk_path, opp_path, av_path, med_path



def show_path(path, state, ax):
    drawstate = state.copy()
    for p in path:
        drawstate[p] = float('inf')
    ax.imshow(drawstate)
    #plt.show()

if __name__ == "__main__":
    state, maskstate, costmap, norisk_path, opp_path, av_path, med_path = run_all_planners(50,100)
    
    # print("ASTAR PATH LEN:", len(astar_path))
    # print("OPPORTUNISTIC PATH LEN:", len(opp_path), 
    #       "\tPROPORTION:", len(opp_path)/len(astar_path),
    #       "\tREPLANS:", opp_ctr)
    # print("RISK-AVERSE PATH LEN:", len(av_path), 
    #       "\tPROPORTION:", len(av_path)/len(astar_path),
    #       "\tREPLANS:", av_ctr)
    # print("MODERATE PATH LEN:", len(med_path), 
    #       "\tPROPORTION:", len(med_path)/len(astar_path),
    #       "\tREPLANS:", opp_ctr)
    
    _, axs = plt.subplots(4, 1, figsize=(6, 9))
    axs = axs.flatten()

    show_path(norisk_path, state, axs[0])
    show_path(opp_path, maskstate, axs[1])
    show_path(av_path, maskstate, axs[2])
    show_path(med_path, maskstate, axs[3])

    plt.show()