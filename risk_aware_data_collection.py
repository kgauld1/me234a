from utils import *
from risk_aware_planner import run_all_planners
import pandas as pd

def get_stats(N,M):
    state, costmap, mask, maskstate, \
        norisk_dstar, norisk_dstar_t, opp_path_dstar, opp_ctr_dstar, opp_dstar_t,\
        av_path_dstar, av_ctr_dstar, av_dstar_t, med_path_dstar, med_ctr_dstar, med_dstar_t,\
        norisk_astar, norisk_astar_t, opp_path_astar, opp_ctr_astar, opp_astar_t,\
        av_path_astar, av_ctr_astar, av_astar_t, med_path_astar, med_ctr_astar, med_astar_t = run_all_planners(N,M)
    stats = [None if norisk_dstar == None else len(norisk_dstar), norisk_dstar_t, 
             None if opp_path_dstar == None else len(opp_path_dstar), opp_ctr_dstar, opp_dstar_t,
             None if av_path_dstar == None else len(av_path_dstar), av_ctr_dstar, av_dstar_t, 
             None if med_path_dstar == None else len(med_path_dstar), med_ctr_dstar, med_dstar_t,
             None if norisk_astar == None else len(norisk_astar), norisk_astar_t, 
             None if opp_path_astar == None else len(opp_path_astar), opp_ctr_astar, opp_astar_t,
             None if av_path_astar == None else len(av_path_astar), av_ctr_astar, av_astar_t, 
             None if med_path_astar == None else len(med_path_astar), med_ctr_astar, med_astar_t]
    return stats

COLUMN_NAMES = ['norisk d* len', 'norisk d* t', 
                'opp path d* len', 'opp ctr d*', 'opp d* t',
                'av path d* len', 'av ctr d*', 'av d* t',
                'med path d* len', 'med ctr d*', 'med d* t',
                'norisk a* len', 'norisk a* t', 
                'opp path a* len', 'opp ctr a*', 'opp a* t',
                'av path a* len', 'av ctr a*', 'av a* t',
                'med path a* len', 'med ctr a*', 'med a* t']

if __name__ == "__main__":
    stats = []
    st = time.perf_counter()
    for i in range(100):
        stats.append(get_stats(50,100))
        if (i+1)%10 == 0: print(i+1, "% done")
    print(time.perf_counter()-st)
    df = pd.DataFrame(np.array(stats), columns=COLUMN_NAMES)
    #print(df)
    df.to_csv('data2.csv')