(define (problem task)
(:domain harpia)
(:objects
    region_1 region_2 region_3 region_4 region_5 region_6 - region
    base_1 base_2 base_3 - base
)
(:init


    (at base_1)








    (picture-goal region_1)
    (picture-goal region_2)
    (picture-goal region_3)
    (picture-goal region_4)


    (= (battery-amount) 100)

    (= (input-amount) 0)


    (= (discharge-rate-battery) 0.042)

    (= (battery-capacity) 100)

    (= (input-capacity) 3)


    (= (distance region_1 region_2) 338.825)
    (= (distance region_1 region_3) 655.288)
    (= (distance region_1 region_4) 659.013)
    (= (distance region_1 base_1) 316.645)
    (= (distance region_1 base_2) 439.803)
    (= (distance region_1 base_3) 1032.04)
    (= (distance region_2 region_1) 338.825)
    (= (distance region_2 region_3) 350.572)
    (= (distance region_2 region_4) 411.83)
    (= (distance region_2 base_1) 121.155)
    (= (distance region_2 base_2) 561.683)
    (= (distance region_2 base_3) 852.903)
    (= (distance region_3 region_1) 655.288)
    (= (distance region_3 region_2) 350.572)
    (= (distance region_3 region_4) 153.287)
    (= (distance region_3 base_1) 339.239)
    (= (distance region_3 base_2) 673.845)
    (= (distance region_3 base_3) 581.705)
    (= (distance region_4 region_1) 659.013)
    (= (distance region_4 region_2) 411.83)
    (= (distance region_4 region_3) 153.287)
    (= (distance region_4 base_1) 353.116)
    (= (distance region_4 base_2) 578.142)
    (= (distance region_4 base_3) 452.557)
    (= (distance base_1 region_1) 316.645)
    (= (distance base_1 region_2) 121.155)
    (= (distance base_1 region_3) 339.239)
    (= (distance base_1 region_4) 353.116)
    (= (distance base_1 base_2) 447.731)
    (= (distance base_1 base_3) 769.595)
    (= (distance base_2 region_1) 439.803)
    (= (distance base_2 region_2) 561.683)
    (= (distance base_2 region_3) 673.845)
    (= (distance base_2 region_4) 578.142)
    (= (distance base_2 base_1) 447.731)
    (= (distance base_2 base_3) 762.125)
    (= (distance base_3 region_1) 1032.04)
    (= (distance base_3 region_2) 852.903)
    (= (distance base_3 region_3) 581.705)
    (= (distance base_3 region_4) 452.557)
    (= (distance base_3 base_1) 769.595)
    (= (distance base_3 base_2) 762.125)

    (= (velocity) 3.5)





    (= (mission-length) 0)

)
(:goal (and
    (taken-image region_1)
    (taken-image region_2)
    (taken-image region_3)
    (taken-image region_4)
    (at base_3)
))
(:metric minimize (mission-length))
)
