
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

Cueing down from goal distance:    4 into depth [1]
                                   3            [1][2]
                                   2            [1][2]
                                   1            [1]
                                   0            

ff: found legal plan as follows
step    0: TAKE_IMAGE REGION_3
        1: GO_TO REGION_3 BASE_1
        2: RECHARGE_BATTERY BASE_1
        3: GO_TO BASE_1 REGION_1
        4: TAKE_IMAGE REGION_1
        5: GO_TO REGION_1 BASE_1
plan cost: 9050.379883

time spent:    0.00 seconds instantiating 267 easy, 0 hard action templates
               0.00 seconds reachability analysis, yielding 35 facts and 53 actions
               0.00 seconds creating final representation with 32 relevant facts, 4 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 12 states, to a max depth of 2
               0.00 seconds total time

