
ff: parsing domain file
domain 'HARPIA' defined
 ... done.
ff: parsing problem file
problem 'TASK' defined
 ... done.


warning: numeric precondition. turning cost-minimizing relaxed plans OFF.

ff: search configuration is Enforced Hill-Climbing, then A*epsilon with weight 5.
Metric is ((1.00*[RF0](MISSION-LENGTH)) - () + 0.00)
COST MINIMIZATION DONE (WITHOUT cost-minimizing relaxed plans).

Cueing down from goal distance:    8 into depth [1][2][3][4][5][6][7]
                                   7            [1]
                                   6            [1]
                                   5            [1]
                                   4            [1]
                                   3            [1]
                                   2            [1]
                                   1            [1]
                                   0            

ff: found legal plan as follows
step    0: GO_TO BASE_3 REGION_5
        1: GO_TO REGION_5 REGION_3
        2: GO_TO REGION_3 REGION_6
        3: GO_TO REGION_6 BASE_3
        4: RECHARGE_BATTERY BASE_3
        5: GO_TO BASE_3 REGION_3
        6: TAKE_IMAGE REGION_3
        7: GO_TO REGION_3 REGION_4
        8: TAKE_IMAGE REGION_4
        9: GO_TO REGION_4 REGION_5
       10: TAKE_IMAGE REGION_5
       11: GO_TO REGION_5 REGION_6
       12: TAKE_IMAGE REGION_6
       13: GO_TO REGION_6 BASE_3
plan cost: 5321.147949

time spent:    0.00 seconds instantiating 93 easy, 0 hard action templates
               0.00 seconds reachability analysis, yielding 24 facts and 84 actions
               0.00 seconds creating final representation with 16 relevant facts, 4 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 285 states, to a max depth of 7
               0.00 seconds total time

