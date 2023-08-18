(define (problem task)
(:domain harpia)
(:objects
    region_1 region_2 region_3 region_4 region_5 region_6 - region
    base_1 base_2 base_3 - base
)
(:init


    (at base_1)








    (picture-goal region_1)


    (= (battery-amount) 100)

    (= (input-amount) 0)


    (= (discharge-rate-battery) 0.042)

    (= (battery-capacity) 100)

    (= (input-capacity) 3)


    (= (distance region_1 base_1) 316.645)
    (= (distance region_1 base_2) 439.803)
    (= (distance region_1 base_3) 1032.04)
    (= (distance base_1 region_1) 316.645)
    (= (distance base_1 base_2) 447.731)
    (= (distance base_1 base_3) 769.595)
    (= (distance base_2 region_1) 439.803)
    (= (distance base_2 base_1) 447.731)
    (= (distance base_2 base_3) 762.125)
    (= (distance base_3 region_1) 1032.04)
    (= (distance base_3 base_1) 769.595)
    (= (distance base_3 base_2) 762.125)

    (= (velocity) 3.5)





    (= (mission-length) 0)

)
(:goal (and
    (taken-image region_1)
    (at base_3)
))
(:metric minimize (mission-length))
)
