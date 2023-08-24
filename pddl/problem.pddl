(define (problem task)
(:domain harpia)
(:objects
    region_1 region_2 region_3 region_4 region_5 region_6 - region
    base_1 base_2 base_3 - base
)
(:init
    (been-at region_1)
    (been-at region_2)
    (been-at region_3)
    (been-at region_4)
    (been-at region_5)
    (been-at region_6)
    (been-at base_3)


    (at base_3)



    (taken-image region_1)
    (taken-image region_2)
    (taken-image region_3)
    (taken-image region_4)
    (taken-image region_5)
    (taken-image region_6)





    (picture-goal region_1)
    (picture-goal region_2)
    (picture-goal region_3)
    (picture-goal region_4)
    (picture-goal region_5)
    (picture-goal region_6)


    (= (battery-amount) 66)

    (= (input-amount) 0)


    (= (discharge-rate-battery) 0.042)

    (= (battery-capacity) 100)

    (= (input-capacity) 3)


    (= (distance region_1 region_2) 139.475)
    (= (distance region_1 region_3) 142.069)
    (= (distance region_1 region_4) 111.994)
    (= (distance region_1 region_5) 236.134)
    (= (distance region_1 region_6) 328.02)
    (= (distance region_1 base_1) 96.6439)
    (= (distance region_1 base_2) 341.897)
    (= (distance region_1 base_3) 84.8273)
    (= (distance region_2 region_1) 139.475)
    (= (distance region_2 region_3) 87.3135)
    (= (distance region_2 region_4) 150.718)
    (= (distance region_2 region_5) 192.152)
    (= (distance region_2 region_6) 270.973)
    (= (distance region_2 base_1) 127.411)
    (= (distance region_2 base_2) 302.362)
    (= (distance region_2 base_3) 105.504)
    (= (distance region_3 region_1) 142.069)
    (= (distance region_3 region_2) 87.3135)
    (= (distance region_3 region_4) 84.7328)
    (= (distance region_3 region_5) 110.707)
    (= (distance region_3 region_6) 197.842)
    (= (distance region_3 base_1) 183.063)
    (= (distance region_3 base_2) 222.71)
    (= (distance region_3 base_3) 62.3309)
    (= (distance region_4 region_1) 111.994)
    (= (distance region_4 region_2) 150.718)
    (= (distance region_4 region_3) 84.7328)
    (= (distance region_4 region_5) 134.442)
    (= (distance region_4 region_6) 224.729)
    (= (distance region_4 base_1) 192.813)
    (= (distance region_4 base_2) 232.846)
    (= (distance region_4 base_3) 48.3779)
    (= (distance region_5 region_1) 236.134)
    (= (distance region_5 region_2) 192.152)
    (= (distance region_5 region_3) 110.707)
    (= (distance region_5 region_4) 134.442)
    (= (distance region_5 region_6) 91.9003)
    (= (distance region_5 base_1) 292.575)
    (= (distance region_5 base_2) 112.006)
    (= (distance region_5 base_3) 151.901)
    (= (distance region_6 region_1) 328.02)
    (= (distance region_6 region_2) 270.973)
    (= (distance region_6 region_3) 197.842)
    (= (distance region_6 region_4) 224.729)
    (= (distance region_6 region_5) 91.9003)
    (= (distance region_6 base_1) 380.899)
    (= (distance region_6 base_2) 47.4811)
    (= (distance region_6 base_3) 243.646)
    (= (distance base_1 region_1) 96.6439)
    (= (distance base_1 region_2) 127.411)
    (= (distance base_1 region_3) 183.063)
    (= (distance base_1 region_4) 192.813)
    (= (distance base_1 region_5) 292.575)
    (= (distance base_1 region_6) 380.899)
    (= (distance base_1 base_2) 404.12)
    (= (distance base_1 base_3) 149.93)
    (= (distance base_2 region_1) 341.897)
    (= (distance base_2 region_2) 302.362)
    (= (distance base_2 region_3) 222.71)
    (= (distance base_2 region_4) 232.846)
    (= (distance base_2 region_5) 112.006)
    (= (distance base_2 region_6) 47.4811)
    (= (distance base_2 base_1) 404.12)
    (= (distance base_2 base_3) 259.892)
    (= (distance base_3 region_1) 84.8273)
    (= (distance base_3 region_2) 105.504)
    (= (distance base_3 region_3) 62.3309)
    (= (distance base_3 region_4) 48.3779)
    (= (distance base_3 region_5) 151.901)
    (= (distance base_3 region_6) 243.646)
    (= (distance base_3 base_1) 149.93)
    (= (distance base_3 base_2) 259.892)

    (= (velocity) 3.5)





    (= (mission-length) 0)

)
(:goal (and
    (at base_3)
))
(:metric minimize (mission-length))
)
