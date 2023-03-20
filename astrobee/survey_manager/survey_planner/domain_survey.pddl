(define (domain survey-manager)
    (:requirements
        :strips
        :typing
        :durative-actions
    )
 
    (:types
    object
        location
        robot - object
        energy-level - object
        data_volume - object
    )
 
    (:predicates
        (robot-not-busy ?r - robot)
        (mutex-moving-robots ?walkingrobot - robot ?cannotwalkrobot - robot)
        ;; only one robot in a location at a time
        (no-robot ?location - location)
        ;; can a robot tranverse this location
        (robot-allowed ?robot - robot ?location - location)
         
        ;; object locations
        (at ?obj - object ?location - location)
 
        ;; connected locations
        (connected ?locA - location ?locB - location)
        (connected ?locB - location ?locC - location)
        (connected ?locC - location ?locD - location)
 
    )
 
 
    ;; move-to
    ;; action is defined (with mutex-moving-robots) assuming only two robots currently
    (:durative-action move-to
        :parameters (?robot - robot )
        :duration (= ?duration 10)
        :condition (and
                    (at start (mutex-moving-robots ?movingrobot ?mutexrobot)) ;; must be defined or robots won't move
                    (at start (robot-not-busy ?movingrobot))
                    (at start (at ?movingrobot ?from))
                    (at end (no-robot ?to))
                    (over all (robot-allowed ?movingrobot ?to))
                    (over all (robot-not-busy ?mutexrobot))
                    (over all (connected ?from ?to)))
        :effect (and
                    (at start (not (robot-not-busy ?movingrobot)))
                    (at start (not (at ?movingrobot ?from)))
                    (at end (not (no-robot ?to)))
                    (at start (no-robot ?from))
                    (at end (robot-not-busy ?movingrobot))
                    (at end (at ?movingrobot ?to))
                )
        )
     
    (:durative-action collecting-panoramas
        :parameters (?robot - robot ?loc - location)
        :duration (= ?duration (robot-collecting-panorama ?robot))
        :condition (and
                    (at start (robot-collecting-panorama ?mutexrobot ?loc)) ;; must be defined or robots won't begin collecting
                    (at start (robot-not-busy ?robot))
                    (over all (robot-busy ?mutexrobot))
        :effect (and
                    (at start (not (robot-collecting-panorama ?mutexrobot ?loc)))
                    (at start (not (robot-not-busy ?robot)))   
                    (at end (robot-collected-panorama ?robot ?loc))))
                )
       (:preferences (preference move-to collecting-panoramas))

)
