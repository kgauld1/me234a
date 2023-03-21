from utils import *
from risk_aware_planner import run_all_planners

def get_stats(N,M,risk_t=-0.1):
    state, costmap, mask, maskstate, \
        norisk_dstar, norisk_dstar_t, opp_path_dstar, opp_ctr_dstar, opp_dstar_t,\
        av_path_dstar, av_ctr_dstar, av_dstar_t, med_path_dstar, med_ctr_dstar, med_dstar_t,\
        norisk_astar, norisk_astar_t, opp_path_astar, opp_ctr_astar, opp_astar_t,\
        av_path_astar, av_ctr_astar, av_astar_t, \
        med_path_astar, med_ctr_astar, med_astar_t = run_all_planners(N,M,risk_t=risk_t)
    
    if opp_path_dstar == None:
        opp_ctr_dstar = None
        opp_dstar_t = None
    if opp_path_astar == None:
        opp_ctr_astar = None
        opp_astar_t = None
    if av_path_dstar == None:
        av_ctr_dstar = None
        av_dstar_t = None
    if av_path_astar == None:
        av_ctr_astar = None
        av_astar_t = None
    if med_path_dstar == None:
        med_ctr_dstar = None
        med_dstar_t = None
    if med_path_astar == None:
        med_ctr_astar = None
        med_astar_t = None

    stats = [None if norisk_dstar == None else len(norisk_dstar), norisk_dstar_t, 
             None if opp_path_dstar == None else len(opp_path_dstar), opp_ctr_dstar, opp_dstar_t,
             None if av_path_dstar == None else len(av_path_dstar), av_ctr_dstar, av_dstar_t, 
             None if med_path_dstar == None else len(med_path_dstar), med_ctr_dstar, med_dstar_t,
             None if norisk_astar == None else len(norisk_astar), norisk_astar_t, 
             None if opp_path_astar == None else len(opp_path_astar), opp_ctr_astar, opp_astar_t,
             None if av_path_astar == None else len(av_path_astar), av_ctr_astar, av_astar_t, 
             None if med_path_astar == None else len(med_path_astar), med_ctr_astar, med_astar_t]
    return stats

COLUMN_NAMES = ["Base D* Path Length","Base D* Time","Opportunistic D* Path Length",
                "Opportunistic D* # Replans","Opportunistic D* Time", "Risk-Averse D* Path Length",
                "Risk-Averse D* # Replans", "Risk-Averse D* Time", "Moderate Aversion D* Path Length",
                "Moderate Aversion D* # Replans", "Moderate Aversion D* Time", "Base A* Path Length",
                "Base A* Time", "Opportunistic A* Path Length", "Opportunistic A* # Replans",
                "Opportunistic A* Time","Risk-Averse A* Path Length", "Risk-Averse A* # Replans",
                "Risk-Averse A* Time", "Moderate Aversion A* Path Length", 
                "Moderate Aversion A* # Replans", "Moderate Aversion A* Time"]

if __name__ == "__main__":
    stats = []
    st = time.perf_counter()
    for i in range(100):
        stats.append(get_stats(50,50,risk_t=-0.1))
        if (i+1)%10 == 0: print(i+1, "% done")
    print(time.perf_counter()-st)
    df = pd.DataFrame(np.array(stats), columns=COLUMN_NAMES)
    #print(df)
    df.to_csv('data/dataneg1.csv')