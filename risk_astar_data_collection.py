from utils import *
from risk_aware_planner import run_all_astar

def get_stats(N,M,risk_t=-0.1,uniform=True):
    state, costmap, mask, start, goal = generate_world(N,M,risk_t=risk_t)
    maskstate = state.copy()
    maskstate[np.where(mask==1)] = RISKY
    if uniform: 
        costmap = np.ones((N,M))
    
    norisk_astar, norisk_astar_t, opp_path_astar, opp_ctr_astar, opp_astar_t,\
        av_path_astar, av_ctr_astar, av_astar_t, \
        med_path_astar, med_ctr_astar, med_astar_t = run_all_astar(state, costmap, maskstate, mask, start, goal)
    
    if opp_path_astar == None:
        opp_ctr_astar = None
        opp_astar_t = None
    if av_path_astar == None:
        av_ctr_astar = None
        av_astar_t = None
    if med_path_astar == None:
        med_ctr_astar = None
        med_astar_t = None

    stats = [None if norisk_astar == None else len(norisk_astar), norisk_astar_t, 
             None if opp_path_astar == None else len(opp_path_astar), opp_ctr_astar, opp_astar_t,
             None if av_path_astar == None else len(av_path_astar), av_ctr_astar, av_astar_t, 
             None if med_path_astar == None else len(med_path_astar), med_ctr_astar, med_astar_t]
    return stats

COLUMN_NAMES = ["Base A* Path Length", "Base A* Time", "Opportunistic A* Path Length", 
                "Opportunistic A* # Replans", "Opportunistic A* Time","Risk-Averse A* Path Length", 
                "Risk-Averse A* # Replans", "Risk-Averse A* Time", "Moderate Aversion A* Path Length", 
                "Moderate Aversion A* # Replans", "Moderate Aversion A* Time"]

if __name__ == "__main__":
    stats = []
    st = time.perf_counter()
    for i in range(100):
        stats.append(get_stats(50,50,risk_t=-0.1,uniform=False))
        if (i+1)%10 == 0: print(i+1, "% done")
    print(time.perf_counter()-st)
    df = pd.DataFrame(np.array(stats), columns=COLUMN_NAMES)
    #print(df)
    df.to_csv('data/data_nonuniform.csv')