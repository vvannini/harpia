
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

Cueing down from goal distance:    3 into depth [1]
                                   2            [1]
                                   1            [1]
                                   0            

ff: found legal plan as follows
step    0: TAKE_IMAGE REGION_2
        1: GO_TO REGION_2 REGION_6
        2: TAKE_IMAGE REGION_6
plan cost: 2270.973145

time spent:    0.00 seconds instantiating 90 easy, 0 hard action templates
               0.00 seconds reachability analysis, yielding 21 facts and 39 actions
               0.00 seconds creating final representation with 20 relevant facts, 4 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 5 states, to a max depth of 1
               0.00 seconds total time

