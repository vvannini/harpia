
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

Cueing down from goal distance:   13 into depth [1]
                                  12            [1][2][3]
                                  11            [1]
                                  10            [1]
                                   9            [1]
                                   8            [1][2]
                                   7            [1]
                                   6            [1][2]
                                   5            [1][2]
                                   4            [1][2][3][4]
                                   3            [1]
                                   2            [1][2][3]
                                   1            [1]
                                   0            

ff: found legal plan as follows
step    0: GO_TO BASE_1 REGION_1
        1: TAKE_IMAGE REGION_1
        2: GO_TO REGION_1 BASE_1
        3: RECHARGE_BATTERY BASE_1
        4: GO_TO BASE_1 REGION_2
        5: TAKE_IMAGE REGION_2
        6: GO_TO REGION_2 REGION_5
        7: TAKE_IMAGE REGION_5
        8: GO_TO REGION_5 BASE_3
        9: RECHARGE_BATTERY BASE_3
       10: GO_TO BASE_3 REGION_6
       11: TAKE_IMAGE REGION_6
       12: GO_TO REGION_6 BASE_3
       13: RECHARGE_BATTERY BASE_3
       14: GO_TO BASE_3 REGION_3
       15: TAKE_IMAGE REGION_3
       16: GO_TO REGION_3 BASE_4
       17: RECHARGE_BATTERY BASE_4
       18: GO_TO BASE_4 REGION_4
       19: TAKE_IMAGE REGION_4
       20: GO_TO REGION_4 BASE_1
       21: RECHARGE_BATTERY BASE_1
       22: GO_TO BASE_1 BASE_3
plan cost: 32054.576172

time spent:    0.00 seconds instantiating 270 easy, 0 hard action templates
               0.00 seconds reachability analysis, yielding 38 facts and 104 actions
               0.00 seconds creating final representation with 38 relevant facts, 4 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 190 states, to a max depth of 4
               0.00 seconds total time

