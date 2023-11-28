(define (problem tests)
    (:domain survey-manager)
    (:objects
        ; intialize all the objects in your world
     JEM_bay_1 JEM_bay_2 JEM_bay_3 JEM_bay_4 JEM_bay_5 JEM_bay_6 JEM_bay_7- location
     bumble queen - robot
    )
    (:init
 
     ;; status
     (robot-not-busy bumble)
     (robot-not-busy queen)
 
     ;; non-static location information
     ;; (no-robot r2_start_location)
     (no-robot JEM_bay_1)
     (no-robot JEM_bay_2)
     (no-robot JEM_bay_3)
     (no-robot JEM_bay_4)
     (no-robot JEM_bay_5)
     (no-robot JEM_bay_6)
     (no-robot JEM_bay_7)
 
 
     ;; static facts
        (robot-at bumble JEM_bay_1)
        (robot-at queen JEM_bay_2)
        (energy-level bumble 100)
        (energy-level queen 100)
        (data_storage_ava bumble 100)
        (data_storage queen 0)       
    )
    ;instead
    ; goal for the task planner to solve
    ; can be multiple
    (:goal (and
        (at queen JEM_bay_7)
        (at bumble paramo)
    ))
 
    (:metric minimize (total-time))
)
        ;Constraints (required margins to maintain for resources remaining, inter-robot constraints such as collision avoidance)
        ;Current state for each robot (position, battery energy available, data volume available)
