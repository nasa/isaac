(define (domain survey-manager)
    (:requirements
        :strips
        :typing
        :durative-actions
        :equality
        :disjunctive-preconditions
        :negative-preconditions
        :quantified-preconditions
    )

    (:types
        location
        robot
        order
        run-number
    )

    (:predicates
        ;; === Static predicates ===
        ;; order-before: Indicates ?order1 is before ?order2. If we are making the order values
        ;; correspond to integers with the usual integer ordering and we provide N possible order
        ;; values, we'll unfortunately need to assert this predicate for (N choose 2) pairs of
        ;; values.
        (order-before ?order1 ?order2 - order)

        ;; move-connected: Indicates a robot can travel from ?from to ?to using a single
        ;; move action. Note that these are interpreted as directed links, so for the usual case
        ;; that you can travel in either direction, you must assert the predicate both ways.
        ;; Neither location can be a berth (use the dock-connected predicate for that). However,
        ;; both bays 6 and 7 should be move-connected to both berth approach points.
        (move-connected ?from ?to - location)

        ;; dock-connected: Indicates a robot can move from ?approach to ?berth using a dock
        ;; action, or from ?berth to ?approach using an undock action.
        (dock-connected ?approach ?berth - location)

        ;; === Dynamic predicates ===
        ;; robot-available: Since a robot can only perform one action at a time in our domain, each
        ;; action grabs this mutex.
        (robot-available ?robot - robot)

        ;; robot-at: Indicates the robot's current position (usually not set during execution of
        ;; motion actions).
        (robot-at ?robot - robot ?location - location)

        ;; location-reserved: Indicates that a robot may be occupying this location and it's not
        ;; safe for other robots to have a conflicting reservation (robots can't reserve the same
        ;; location and can't reserve adjacent bays while flying). In the initial state, the robot
        ;; must assert a location-reserved predicate for its starting location as well as robot-at.
        (location-reserved ?location - location)

        ;; robot-order: Indicates the order of the last action executed by ?robot. Later actions
        ;; must not have a lower order. Each robot must have order o-init in the initial state, and
        ;; order o-init should not be used in goals. The special order o-any can be used to indicate
        ;; a goal that has no ordering constraints relative to other goals (experimental).
        (robot-order ?robot - robot ?order - order)

        ;; === Goal predicates ===
        ;; completed-panorama: The goal to add if you want the plan to include collecting a
        ;; panorama. For now, goals specify ?robot and ?order parameters that constrain
        ;; multi-robot task allocation and task ordering.
        (completed-panorama
            ?robot - robot
            ?order - order
            ?location - location
            ?run-number - run-number
        )

        ;; completed-stereo: The goal to add if you want the plan to include collecting a stereo
        ;; survey. For now, goals specify ?robot and ?order parameters that constrain multi-robot
        ;; task allocation and task ordering. (Note that right now, we only reserve the locations
        ;; ?from and ?to... this may be ok if they always bracket the bays included in the survey.)
        (completed-stereo
            ?robot - robot
            ?order - order
            ?from ?to - location  ;; Start and end of trajectory
            ?run-number - run-number
        )
    )

    (:durative-action dock
        :parameters (?robot - robot ?approach ?berth - location)
        :duration (= ?duration 30)
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check parameters make sense
                (at start (robot-at ?robot ?approach))
                (at start (dock-connected ?approach ?berth))

                ;; Check collision avoidance
                (at start (not (location-reserved ?berth)))
            )
        :effect
            (and
                ;; Grab and release robot mutex
                (at start (not (robot-available ?robot)))
                (at end (robot-available ?robot))

                ;; Grab and release reserved locations
                (at start (location-reserved ?berth))
                (at end (not (location-reserved ?approach)))

                ;; Update robot location
                (at start (not (robot-at ?robot ?approach)))
                (at end (robot-at ?robot ?berth))
            )
    )

    (:durative-action undock
        :parameters (?robot - robot ?berth ?approach - location)
        :duration (= ?duration 30)
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check parameters make sense
                (at start (robot-at ?robot ?berth))
                (at start (dock-connected ?approach ?berth))

                ;; Check collision avoidance
                (at start (not (location-reserved ?approach)))
                (at start
                    (forall (?nearby - location)
                        (not
                            (and
                                (location-reserved ?nearby)
                                (move-connected ?nearby ?approach)))))
            )
        :effect
            (and
                ;; Grab and release robot mutex
                (at start (not (robot-available ?robot)))
                (at end (robot-available ?robot))

                ;; Grab and release reserved locations
                (at start (location-reserved ?approach))
                (at end (not (location-reserved ?berth)))

                ;; Update robot location
                (at start (not (robot-at ?robot ?berth)))
                (at end (robot-at ?robot ?approach))
            )
    )

    (:durative-action move
        :parameters (?robot - robot ?from ?to - location)
        :duration (= ?duration 20)
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check parameters make sense
                (at start (robot-at ?robot ?from))
                (at start (move-connected ?from ?to))

                ;; Check collision avoidance
                (at start (not (location-reserved ?to)))
                (at start
                    (forall (?nearby - location)
                        (not
                            (and
                                (location-reserved ?nearby)
                                (move-connected ?nearby ?to)))))
            )
        :effect
            (and
                ;; Grab and release robot mutex
                (at start (not (robot-available ?robot)))
                (at end (robot-available ?robot))

                ;; Grab and release reserved locations
                (at start (location-reserved ?to))
                (at end (not (location-reserved ?from)))

                ;; Update robot location
                (at start (not (robot-at ?robot ?from)))
                (at end (robot-at ?robot ?to))
            )
    )

    (:durative-action panorama
        :parameters
            (
                ?robot - robot
                ?old-order ?order - order
                ?location - location
                ?run-number - run-number
            )
        :duration (= ?duration 900)  ;; 15 minutes
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check order
                (at start (robot-order ?robot ?old-order))
                (at start (not (order-before ?order ?old-order)))

                ;; Check parameters make sense
                (at start (robot-at ?robot ?location))
            )
        :effect
            (and
                ;; Grab and release robot mutex
                (at start (not (robot-available ?robot)))
                (at end (robot-available ?robot))

                ;; Update order
                (at start (not (robot-order ?robot ?old-order)))
                (at end (robot-order ?robot ?order))

                ;; Mark success
                (at end (completed-panorama ?robot ?order ?location ?run-number))
            )
    )

    (:durative-action stereo
        :parameters
            (
                ?robot - robot
                ?old-order ?order - order
                ?from ?to - location  ;; Start and end of the trajectory
                ?run-number - run-number
            )
        :duration (= ?duration 600)  ;; 10 minutes
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check order
                (at start (robot-order ?robot ?old-order))
                (at start (not (order-before ?order ?old-order)))

                ;; Check parameters make sense
                (at start (robot-at ?robot ?from))

                ;; Check collision avoidance (note we only check/reserve the locations ?from and
                ;; ?to... this may be ok if they always bracket the bays included in the survey)
                (at start (not (location-reserved ?to)))
                (at start
                    (forall (?nearby - location)
                        (not
                            (and
                                (location-reserved ?nearby)
                                (move-connected ?nearby ?to)))))
            )
        :effect
            (and
                ;; Grab and release robot mutex
                (at start (not (robot-available ?robot)))
                (at end (robot-available ?robot))

                ;; Update order
                (at start (not (robot-order ?robot ?old-order)))
                (at end (robot-order ?robot ?order))

                ;; Grab and release reserved locations
                (at start (location-reserved ?to))
                (at end (not (location-reserved ?from)))

                ;; Update robot location
                (at start (not (robot-at ?robot ?from)))
                (at end (robot-at ?robot ?to))

                ;; Mark success
                (at end (completed-stereo ?robot ?order ?from ?to ?run-number))
            )
    )
)
