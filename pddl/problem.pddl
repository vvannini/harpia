(define (problem task)
(:domain harpia)
(:objects
    region_1 region_2 region_3 region_4 region_5 region_6 region_7 region_8 region_9 region_10 region_11 region_12 - region
    base_1 base_2 base_3 base_4 - base
)
(:init


    (at base_1)








    (picture-goal region_1)
    (picture-goal region_2)
    (picture-goal region_3)
    (picture-goal region_4)
    (picture-goal region_5)
    (picture-goal region_6)


    (= (battery-amount) 100)

    (= (input-amount) 0)


    (= (discharge-rate-battery) 0.042)

    (= (battery-capacity) 100)

    (= (input-capacity) 3)


    (= (distance region_1 region_2) 2659.31)
    (= (distance region_1 region_3) 2474.48)
    (= (distance region_1 region_4) 2981.27)
    (= (distance region_1 region_5) 3855.53)
    (= (distance region_1 region_6) 3065.19)
    (= (distance region_1 base_1) 2137.48)
    (= (distance region_1 base_2) 1421.39)
    (= (distance region_1 base_3) 4930.21)
    (= (distance region_1 base_4) 3092.56)
    (= (distance region_2 region_1) 2659.31)
    (= (distance region_2 region_3) 1352.25)
    (= (distance region_2 region_4) 2627.34)
    (= (distance region_2 region_5) 1231.98)
    (= (distance region_2 region_6) 498.419)
    (= (distance region_2 base_1) 1835.59)
    (= (distance region_2 base_2) 1870.26)
    (= (distance region_2 base_3) 2292.66)
    (= (distance region_2 base_4) 1043.42)
    (= (distance region_3 region_1) 2474.48)
    (= (distance region_3 region_2) 1352.25)
    (= (distance region_3 region_4) 1275.89)
    (= (distance region_3 region_5) 1922.1)
    (= (distance region_3 region_6) 1307.94)
    (= (distance region_3 base_1) 2775.42)
    (= (distance region_3 base_2) 1123.67)
    (= (distance region_3 base_3) 3241.38)
    (= (distance region_3 base_4) 821.239)
    (= (distance region_4 region_1) 2981.27)
    (= (distance region_4 region_2) 2627.34)
    (= (distance region_4 region_3) 1275.89)
    (= (distance region_4 region_5) 2990.49)
    (= (distance region_4 region_6) 2525.8)
    (= (distance region_4 base_1) 3916.24)
    (= (distance region_4 base_2) 1626.99)
    (= (distance region_4 base_3) 4308.77)
    (= (distance region_4 base_4) 1885.64)
    (= (distance region_5 region_1) 3855.53)
    (= (distance region_5 region_2) 1231.98)
    (= (distance region_5 region_3) 1922.1)
    (= (distance region_5 region_4) 2990.49)
    (= (distance region_5 region_6) 790.986)
    (= (distance region_5 base_1) 2920.99)
    (= (distance region_5 base_2) 2851.53)
    (= (distance region_5 base_3) 1327.84)
    (= (distance region_5 base_4) 1126.19)
    (= (distance region_6 region_1) 3065.19)
    (= (distance region_6 region_2) 498.419)
    (= (distance region_6 region_3) 1307.94)
    (= (distance region_6 region_4) 2525.8)
    (= (distance region_6 region_5) 790.986)
    (= (distance region_6 base_1) 2321.42)
    (= (distance region_6 base_2) 2099.41)
    (= (distance region_6 base_3) 2003.3)
    (= (distance region_6 base_4) 710.664)
    (= (distance base_1 region_1) 2137.48)
    (= (distance base_1 region_2) 1835.59)
    (= (distance base_1 region_3) 2775.42)
    (= (distance base_1 region_4) 3916.24)
    (= (distance base_1 region_5) 2920.99)
    (= (distance base_1 region_6) 2321.42)
    (= (distance base_1 base_2) 2474.12)
    (= (distance base_1 base_3) 3513.11)
    (= (distance base_1 base_4) 2828.67)
    (= (distance base_2 region_1) 1421.39)
    (= (distance base_2 region_2) 1870.26)
    (= (distance base_2 region_3) 1123.67)
    (= (distance base_2 region_4) 1626.99)
    (= (distance base_2 region_5) 2851.53)
    (= (distance base_2 region_6) 2099.41)
    (= (distance base_2 base_1) 2474.12)
    (= (distance base_2 base_3) 4100.55)
    (= (distance base_2 base_4) 1872.06)
    (= (distance base_3 region_1) 4930.21)
    (= (distance base_3 region_2) 2292.66)
    (= (distance base_3 region_3) 3241.38)
    (= (distance base_3 region_4) 4308.77)
    (= (distance base_3 region_5) 1327.84)
    (= (distance base_3 region_6) 2003.3)
    (= (distance base_3 base_1) 3513.11)
    (= (distance base_3 base_2) 4100.55)
    (= (distance base_3 base_4) 2454)
    (= (distance base_4 region_1) 3092.56)
    (= (distance base_4 region_2) 1043.42)
    (= (distance base_4 region_3) 821.239)
    (= (distance base_4 region_4) 1885.64)
    (= (distance base_4 region_5) 1126.19)
    (= (distance base_4 region_6) 710.664)
    (= (distance base_4 base_1) 2828.67)
    (= (distance base_4 base_2) 1872.06)
    (= (distance base_4 base_3) 2454)

    (= (velocity) 3.5)





    (= (mission-length) 0)

)
(:goal (and
    (taken-image region_1)
    (taken-image region_2)
    (taken-image region_3)
    (taken-image region_4)
    (taken-image region_5)
    (taken-image region_6)
    (at base_3)
))
(:metric minimize (mission-length))
)
