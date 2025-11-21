(define (domain exploration)
    (:requirements :strips :typing :adl)

    (:types
        position area
    )

    (:predicates
        (area_explored ?ar - area)
        (anomaly_detected ?an_pos - position) ; Anomaly is just a position for now
    )

    (:action explore
        :parameters (?ar - area ?an_pos - position)
        :precondition (and 
            (not (area_explored ?ar)) 
            (not (anomaly_detected ?an_pos))
        )
        :effect (and (area_explored ?ar))
    )

    (:action investigate
        :parameters (?an_pos - position)
        :precondition (and (anomaly_detected ?an_pos))
        :effect (and (not (anomaly_detected ?an_pos)))
    )
    
)
    

    ; (:durative-action explore
    ;     :parameters (?ar - area ?an_pos - position)
    ;     :duration (= ?duration 1)
    ;     :condition (and
    ;         (at start (not (area_explored ?ar)))
    ;         (over all (not (anomaly_detected ?an_pos)))
    ;     )
    ;     :effect (and
    ;         (at end (area_explored ?ar))
    ;     )
    ; )
    

    ; (:durative-action investigate
    ;     :parameters (?an_pos - position)
    ;     :duration (= ?duration 1)
    ;     :condition (and
    ;         (at start (anomaly_detected ?an_pos))
    ;     )
    ;     :effect (and
    ;         (at end (not (anomaly_detected ?an_pos)))
    ;     )
    ; )
