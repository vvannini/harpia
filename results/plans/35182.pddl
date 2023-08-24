
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

Cueing down from goal distance:    9 into depth [1]
                                   8            [1][2][3][4][5][6][7]
                                   7            [1]
                                   6            [1]
                                   5            [1]
                                   4            [1]
                                   3            [1]
                                   2            [1]
                                   1            [1]
                                   0            

ff: found legal plan as follows
step    0: GO_TO REGION_2 REGION_3
        1: GO_TO REGION_3 REGION_4
        2: GO_TO REGION_4 REGION_3
        3: GO_TO REGION_3 REGION_6
        4: GO_TO REGION_6 BASE_3
        5: RECHARGE_BATTERY BASE_3
        6: GO_TO BASE_3 REGION_3
        7: TAKE_IMAGE REGION_3
        8: GO_TO REGION_3 REGION_4
        9: TAKE_IMAGE REGION_4
       10: GO_TO REGION_4 REGION_5
       11: TAKE_IMAGE REGION_5
       12: GO_TO REGION_5 REGION_6
       13: TAKE_IMAGE REGION_6
       14: GO_TO REGION_6 BASE_3
plan cost: 5315.318848

time spent:    0.00 seconds instantiating 93 easy, 0 hard action templates
               0.00 seconds reachability analysis, yielding 24 facts and 84 actions
               0.00 seconds creating final representation with 20 relevant facts, 4 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.01 seconds searching, evaluating 715 states, to a max depth of 7
               0.01 seconds total time

