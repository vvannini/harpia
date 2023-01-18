Number of literals: 30
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
89% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 19.000
b (18.000 | 125.658)b (16.000 | 411.373)b (15.000 | 508.181)b (13.000 | 793.896)b (12.000 | 894.060)b (10.000 | 1179.774)b (9.000 | 1223.572)b (7.000 | 1509.286)b (6.000 | 1620.148)b (5.000 | 2021.008)b (3.000 | 2060.636)b (1.000 | 2376.911);;;; Solution Found
; States evaluated: 146
; Cost: 2407.472
; Time 0.54
0.000: (go_to_picture base_2 region_1)  [125.658]
125.659: (take_image region_1)  [285.714]
411.374: (go_to_picture region_1 region_2)  [96.807]
508.181: (take_image region_2)  [285.714]
793.897: (go_to_picture region_2 region_3)  [100.163]
894.060: (take_image region_3)  [285.714]
1179.775: (go_to_picture region_3 region_4)  [43.796]
1223.572: (take_image region_4)  [285.714]
1509.287: (go_to_picture region_4 region_5)  [110.861]
1620.148: (take_image region_5)  [285.714]
1905.863: (go_to_base region_5 base_3)  [115.144]
2021.009: (recharge_battery base_3)  [39.627]
2060.637: (go_to_picture base_3 region_6)  [30.560]
2091.197: (take_image region_6)  [285.714]
2376.912: (go_to_base region_6 base_3)  [30.560]
