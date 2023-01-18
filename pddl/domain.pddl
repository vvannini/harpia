(define (domain harpia)

    (:requirements  :typing  :strips  :disjunctive-preconditions  :equality )

    (:types
        region - object
        base - region)


   
    (:functions
    

        ;; Variavel q controla bateria em porcentagem
        (battery-amount)
        ;; quantidade de insumo
        (input-amount)
        ;;velocidade de carregar a bateria em porcentagem por segundos
        (recharge-rate-battery)
        ;;velocidade de descarregar a bateria
        (discharge-rate-battery)
        ;;capacidade maxima bateria
        (battery-capacity)
        ;;capacidade maxima de insumo
        (input-capacity)
        ;;velocidade de reabastecer o insumo
        (recharge-rate-input)
        ;;distancia entre regioes em metros
        (distance ?from-region - region ?to-region - region)
        ;;velocidade em m/s
        (velocity)
        (picture-path-len ?region - region)
        (pulverize-path-len ?region - region)
        (total-goals)
        (goals-achived)
        (mission-length)

    )

     (:predicates
    
        (been-at ?region - region)
        ;;se esta carregando um insumo
        (carry)  
        ;;esta em uma regiao
        (at ?region - region)
        ;; se pode pulverizar
        (can-spray)
        ;;se pode carregar/descarregar
        (can-recharge)
        ;se já tirou a foto
        (taken-image ?region - region)
        ;se pulverizou
        (pulverized ?region - region)
        ; (canGo)
        (can-take-pic)
        (its-not-base ?region - region)
        (pulverize-goal ?region - region)
        (picture-goal ?region - region)
        (hw-ready ?from - region ?to - region)

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
            (> (battery-amount) (+ (* (/ (distance ?from-region ?to-region) (velocity)) (discharge-rate-battery)) 15))
        )
        :effect (and 
                (not (at ?from-region))
                (been-at ?to-region)
                (at ?to-region)
                (decrease (battery-amount ) 
                      (*
                          (/
                              (distance ?from-region ?to-region)
                              (velocity)
                          )
                          (discharge-rate-battery)
                      )
          
                )
                (increase (mission-length) (distance ?from-region ?to-region))
                )
    )
    
    (:action take_image
        :parameters (
            ?region - region
        )
        :precondition(and
            (at ?region)
            (picture-goal ?region)
            (> (battery-amount) 
                (*
                    (/
                        1000
                        (velocity)
                    )
                    (discharge-rate-battery)
                )
            )
       )
        :effect(and
            (taken-image ?region)
            (increase (mission-length) 1000)
            (decrease (battery-amount) 
                (*
                    (/
                        1000
                        (velocity)
                    )
                    (discharge-rate-battery)
                )
            )
        )
    )
    (:action pulverize_region
        :parameters (
            ?region - region)
        :precondition(and
            (at ?region)
            (pulverize-goal ?region)
            (> (input-amount) 0)
            (> (battery-amount) 
                (*
                    (/
                        314
                        (velocity)
                    )
                    (discharge-rate-battery)
                )
            )
       )
        :effect(and
            (pulverized ?region)
            (increase (mission-length) 314)
            (decrease (input-amount) 1)
            (decrease (battery-amount) 
                (*
                    (/
                        314
                        (velocity)
                    )
                    (discharge-rate-battery)
                )
            )
        )
    )
    (:action recharge_battery
        :parameters (?base - base)
        :precondition (and
            (at ?base)
            ;(< (battery-amount) 60)
        )
        :effect 
        (and
            (assign (battery-amount) (battery-capacity))
        )
    )

    (:action recharge_input
        :parameters (?base - base)
        :precondition (and
            (at ?base)
            (< (input-amount) (/ (input-capacity) 2))
        )
        :effect 
        (and
            (assign (input-amount) (input-capacity))
        )
    )
)