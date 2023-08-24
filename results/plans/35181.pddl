
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

Cueing down from goal distance:   11 into depth [1]
                                  10            [1]
                                   9            [1]
                                   8            [1]
                                   7            [1]
                                   6            [1]
                                   5            [1]
                                   4            [1]
                                   3            [1]
                                   2            [1]
                                   1            [1]
                                   0            

ff: found legal plan as follows
step    0: GO_TO REGION_1 REGION_2
        1: TAKE_IMAGE REGION_2
        2: GO_TO REGION_2 REGION_3
        3: TAKE_IMAGE REGION_3
        4: GO_TO REGION_3 REGION_4
        5: TAKE_IMAGE REGION_4
        6: GO_TO REGION_4 REGION_5
        7: TAKE_IMAGE REGION_5
        8: GO_TO REGION_5 REGION_6
        9: TAKE_IMAGE REGION_6
       10: GO_TO REGION_6 BASE_3
plan cost: 5781.509766

time spent:    0.00 seconds instantiating 93 easy, 0 hard action templates
               0.00 seconds reachability analysis, yielding 24 facts and 84 actions
               0.00 seconds creating final representation with 22 relevant facts, 4 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 22 states, to a max depth of 1
               0.00 seconds total time

