(define (problem task)
(:domain harpia)
(:objects
    region_1 region_2 region_3 region_4 region_5 region_6 - region
    base_1 base_2 base_3 - base
)
(:init


    (at base_1)








    (picture-goal region_6)


    (= (battery-amount) 100)

    (= (input-amount) 0)


    (= (discharge-rate-battery) 0.042)

    (= (battery-capacity) 100)

    (= (input-capacity) 3)


    (= (distance region_6 base_1) 380.899)
    (= (distance region_6 base_2) 47.4811)
    (= (distance region_6 base_3) 243.646)
    (= (distance base_1 region_6) 380.899)
    (= (distance base_1 base_2) 404.12)
    (= (distance base_1 base_3) 149.93)
    (= (distance base_2 region_6) 47.4811)
    (= (distance base_2 base_1) 404.12)
    (= (distance base_2 base_3) 259.892)
    (= (distance base_3 region_6) 243.646)
    (= (distance base_3 base_1) 149.93)
    (= (distance base_3 base_2) 259.892)

    (= (velocity) 3.5)





    (= (mission-length) 0)

)
(:goal (and
    (taken-image region_6)
    (at base_1)
))
(:metric minimize (mission-length))
)
