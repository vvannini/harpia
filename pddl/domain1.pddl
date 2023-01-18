(define (domain harpia)

    (:requirements  :typing  :strips  :disjunctive-preconditions  :equality )

    (:types
        region - object
        base - region)


   
    (:functions
    
        ;; Variavel q controla bateria em porcentagem
        (batteryAmount)
        ;; quantidade de insumo
        (inputAmount)
        ;;velocidade de carregar a bateria em porcentagem por segundos
        (rechargeRateBattery)
        ;;velocidade de descarregar a bateria
        (dischargeRateBattery)
        ;;capacidade maxima bateria
        (batteryCapacity)
        ;;capacidade maxima de insumo
        (inputCapacity)
        ;;velocidade de reabastecer o insumo
        (rechargeRateInput)
        ;;distancia entre regioes em metros
        (distance ?from-region - region ?to-region - region)
        ;;velocidade em m/s
        (velocity)
        (picturePathLen ?region - region)
        (pulverizePathLen ?region - region)
        (totalGoals)
        (goalsAchived)
        (missionLength)

    )

     (:predicates
    
        (beenAt ?region - region)
        ;;se esta carregando um insumo
        (carry)  
        ;;esta em uma regiao
        (at ?region - region)
        ;; se pode pulverizar
        (canSpray)
        ;;se pode carregar/descarregar
        (canRecharge)
        ;se já tirou a foto
        (takenImage ?region - region)
        ;se pulverizou
        (pulverized ?region - region)
        ; (canGo)
        (canTakePic)
        (itsNotBase ?region - region)
        (pulverizeGoal ?region - region)
        (pictureGoal ?region - region)
        (hwTeady ?from - region ?to - region)

        ; (can-go-to-base)
        ; (has-pulverize-goal)
        ; (has-picture-goal)
        ; (at-move)
    
    )


    (:action go_to
        :parameters (
             ?from-region - region 
             ?to-region - region)
        :precondition (and
            (at ?from-region)
            (> (batteryAmount) (+ (* (/ (distance ?from-region ?to-region) (velocity)) (dischargeRateBattery)) 15))
        )
        :effect (and 
                (not (at ?from-region))
                (beenAt ?to-region)
                (at ?to-region)
                (decrease (batteryAmount ) 
                      (*
                          (/
                              (distance ?from-region ?to-region)
                              (velocity)
                          )
                          (dischargeRateBattery)
                      )
          
                )
                (increase (missionLength) (distance ?from-region ?to-region))
                )
    )
    
    (:action take_image
        :parameters (
            ?region - region
        )
        :precondition(and
            (at ?region)
            (pictureGoal ?region)
            (> (batteryAmount) 
                (*
                    (/
                        1000
                        (velocity)
                    )
                    (dischargeRateBattery)
                )
            )
       )
        :effect(and
            (takenImage ?region)
            (increase (missionLength) 1000)
            (decrease (batteryAmount) 
                (*
                    (/
                        1000
                        (velocity)
                    )
                    (dischargeRateBattery)
                )
            )
        )
    )
    (:action pulverize_region
        :parameters (
            ?region - region)
        :precondition(and
            (at ?region)
            (pulverizeGoal ?region)
            (> (inputAmount) 0)
            (> (batteryAmount) 
                (*
                    (/
                        314
                        (velocity)
                    )
                    (dischargeRateBattery)
                )
            )
       )
        :effect(and
            (pulverized ?region)
            (increase (missionLength) 314)
            (decrease (inputAmount) 1)
            (decrease (batteryAmount) 
                (*
                    (/
                        314
                        (velocity)
                    )
                    (dischargeRateBattery)
                )
            )
        )
    )
    (:action recharge_battery
        :parameters (?base - base)
        :precondition (and
            (at ?base)
            ;(< (batteryAmount) 60)
        )
        :effect 
        (and
            (assign (batteryAmount) (batteryCapacity))
        )
    )

    (:action recharge_input
        :parameters (?base - base)
        :precondition (and
            (at ?base)
            (< (inputAmount) (/ (inputCapacity) 2))
        )
        :effect 
        (and
            (assign (inputAmount) (inputCapacity))
        )
    )
)