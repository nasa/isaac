(define (domain survey-manager)
    (:requirements
        :strips
        :typing
        :durative-actions
        :fluents
    )

    (:types
        location
        robot
        order
    )

    (:predicates
        ;; === Static predicates ===
        ;; move-connected: Indicates a robot can travel from ?from to ?to using a single
        ;; move action. Note that these are interpreted as directed links, so for the usual case
        ;; that you can travel in either direction, you must assert the predicate both ways.
        ;; Neither location can be a berth (use the dock-connected predicate for that). However,
        ;; both bays 6 and 7 should be move-connected to both berth approach points.
        (move-connected ?from ?to - location)

        ;; location-real: Indicates a location is a real place the robot can fly to. We've added
        ;; bogus locations bay0 and bay8 to the problem instance to satisfy the implicit assumption
        ;; of the collision checking that every bay has two neighbors. We assert location-real for
        ;; the other locations and use it as a precondition on moves so the planner can't
        ;; accidentally fly to a bogus location.
        (location-real ?location - location)

        ;; dock-connected: Indicates a robot can move from ?approach to ?berth using a dock
        ;; action, or from ?berth to ?approach using an undock action.
        (dock-connected ?approach ?berth - location)

        ;; robots-different: Indicates a != b. Needs to be expressed as a positive predicate
        ;; so it can be used as a precondition. Must be asserted in both directions.
        (robots-different ?a ?b - robot)

        ;; locations-different: Indicates a != b. Needs to be expressed as a positive predicate
        ;; so it can be used as a precondition. Must be asserted in both directions.
        (locations-different ?a ?b - location)

        ;; === Dynamic predicates ===
        ;; robot-available: Since a robot can only perform one action at a time in our domain, each
        ;; action grabs this mutex. In the initial state, both robots should be available.
        (robot-available ?robot - robot)

        ;; robot-at: Indicates the robot's current position (usually not set during execution of
        ;; motion actions). In the initial state, both robots should have robot-at set for their
        ;; initial locations.
        (robot-at ?robot - robot ?location - location)

        ;; location-available: Indicates that no robot has reserved the location. When stationary,
        ;; each robot reserves its current location. During a move, it reserves both its ?from
        ;; location and its ?to location. Collision avoidance checks prevent robots from reserving
        ;; the same location or reserving adjacent bays while flying. Note: It might be more natural
        ;; to express this as a location-reserved predicate with the opposite boolean sense, but we
        ;; need it to be this way so we can use it as a precondition without negating it.  In the
        ;; initial state, we must mark location-available for all locations, real or bogus, except
        ;; the initial robot locations.
        (location-available ?location - location)

        ;; need-stereo: If you add a completed-stereo goal, you must also add a need-stereo
        ;; predicate with identical parameters to the initial state. This is part of a hack that
        ;; greatly improves planner performance. The need-stereo predicate has been made part of the
        ;; preconditions of the stereo action, and one of its effects is to clear the
        ;; predicate. Therefore, the planner won't waste time trying to execute stereo actions that
        ;; the user didn't explicitly request. Without this hack, the planner run time blows up.
        (need-stereo ?robot - robot ?order - order ?base ?bound - location)

        ;; === Goal predicates ===
        ;; completed-panorama: The goal to add if you want the plan to include collecting a
        ;; panorama. For now, goals specify ?robot and ?order parameters that constrain
        ;; multi-robot task allocation and task ordering.
        (completed-panorama ?robot - robot ?order - order ?location - location)

        ;; completed-stereo: The goal to add if you want the plan to include collecting a stereo
        ;; survey. For now, goals specify ?robot and ?order parameters that constrain multi-robot
        ;; task allocation and task ordering. The current model for stereo surveys assumes the robot
        ;; starts and ends the survey at the same location called ?base (these locations only need
        ;; to be same to the effective precision modeled in the planner, "less than a bay apart").
        ;; The ?bound argument indicates the other end of the interval covered by the survey and is
        ;; used for collision checking. It's assumed that ?base and ?bound are not adjacent
        ;; locations. If future stereo surveys violate these assumptions the model will need to be
        ;; revisited.
        (completed-stereo ?robot - robot ?order - order ?base ?bound - location)

	;; completed-let-other-robot-reach: The goal to add if you want one robot to wait for the
	;; other to reach a certain location before pursuing its remaining goals (ones with larger
	;; ?order values). This basically enables a user to provide a specific kind of
	;; between-robots ordering hint to the planner.
	(completed-let-other-robot-reach ?robot - robot ?order - order ?loc - location)
    )

    (:functions
        ;; === Static numeric fluents ===
        ;; order-identity: An identity operator that maps from a symbolic order like o0 to its
        ;; corresponding numeric value 0.
        (order-identity ?order - order)

        ;; === Dynamic numeric fluents ===
        ;; robot-order: Indicates the order of the last action executed by ?robot. Later actions
        ;; must not have a lower ?order (this only applies to the panorama and stereo actions that
        ;; take an ?order parameter). In the initial state, each robot must have order -1.
        (robot-order ?robot - robot)
    )

    (:durative-action dock
        :parameters (?robot - robot ?from ?to - location)  ;; from bay7 to berth1 or berth2
        :duration (= ?duration 30)
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check parameters make sense
                (at start (robot-at ?robot ?from))
                (at start (dock-connected ?from ?to))

                ;; Check collision avoidance
                (at start (location-available ?to))
                ; Don't need to check berth neighbors
            )
        :effect
            (and
                ;; Grab and release robot mutex
                (at start (not (robot-available ?robot)))
                (at end (robot-available ?robot))

                ;; Grab and release reserved locations
                (at start (not (location-available ?to)))
                (at end (location-available ?from))

                ;; Update robot location
                (at start (not (robot-at ?robot ?from)))
                (at end (robot-at ?robot ?to))
            )
    )

    (:durative-action undock
        :parameters (
            ?robot - robot
            ?from ?to - location  ;; from berth1 or berth2 to bay7
            ?check1 ?check2 - location  ;; neighbors of ?to to check for collision avoidance
        )
        :duration (= ?duration 45)
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check parameters make sense
                (at start (robot-at ?robot ?from))
                (at start (dock-connected ?to ?from))
                (at start (location-real ?to))

                ;(at start (robot-can-undock-now ?robot))

                ;; Check collision avoidance
                (at start (location-available ?to))
                (at start (locations-different ?check1 ?check2))
                (at start (move-connected ?check1 ?to))
                (at start (move-connected ?check2 ?to))
                (at start (location-available ?check1))
                (at start (location-available ?check2))
            )
        :effect
            (and
                ;; Grab and release robot mutex
                (at start (not (robot-available ?robot)))
                (at end (robot-available ?robot))

                ;; Grab and release reserved locations
                (at start (not (location-available ?to)))
                (at end (location-available ?from))

                ;; Update robot location
                (at start (not (robot-at ?robot ?from)))
                (at end (robot-at ?robot ?to))
            )
    )

    (:durative-action move
        :parameters (
            ?robot - robot
            ?from ?to - location
            ?check - location  ;; neighbor of ?to to check for collision avoidance
        )
        :duration (= ?duration 20)
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check parameters make sense
                (at start (robot-at ?robot ?from))
                (at start (move-connected ?from ?to))
                (at start (location-real ?to))

                ;; Check collision avoidance
                (at start (location-available ?to))
                ;; In general when flying to ?to we collision check whether ?to or either of its
                ;; neighbors are reserved (by the other robot). In this case, one of the neighbors
                ;; of ?to is ?from, which can't be reserved by the other robot since this robot
                ;; previously reserved it. Therefore, we only need to check if the other neighbor
                ;; (?check) is reserved.
                (at start (locations-different ?check ?from))
                (at start (move-connected ?check ?to))
                (at start (location-available ?check))
            )
        :effect
            (and
                ;; Grab and release robot mutex
                (at start (not (robot-available ?robot)))
                (at end (robot-available ?robot))

                ;; Grab and release reserved locations
                (at start (not (location-available ?to)))
                (at end (location-available ?from))

                ;; Update robot location
                (at start (not (robot-at ?robot ?from)))
                (at end (robot-at ?robot ?to))
            )
    )

    (:durative-action panorama
        :parameters
            (
                ?robot - robot
                ?order - order
                ?location - location
            )
        ;; ~13 minutes, per https://babelfish.arc.nasa.gov/confluence/display/FFOPS/ISAAC+Phase+1X+Activity+9+Ground+Procedure
        :duration (= ?duration 780)
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check order
                (at start (< (robot-order ?robot) (order-identity ?order)))

                ;; Check parameters make sense
                (at start (robot-at ?robot ?location))
            )
        :effect
            (and
                ;; Grab and release robot mutex
                (at start (not (robot-available ?robot)))
                (at end (robot-available ?robot))

                ;; Update order
                (at end (assign (robot-order ?robot) (order-identity ?order)))

                ;; Mark success
                (at end (completed-panorama ?robot ?order ?location))
            )
    )

    (:durative-action stereo
        :parameters
            (
                ?robot - robot
                ?order - order
                ;; ?base: The bay where we start and also stop. ?bound: The other end of the survey
                ?base ?bound - location
                ;; ?check1 and ?check2: Planner-selected neighbors of ?bound for collision check
                ?check1 ?check2 - location
            )
        :duration (= ?duration 600)  ;; 10 minutes
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check order
                (at start (< (robot-order ?robot) (order-identity ?order)))

                ;; Check parameters make sense
                (at start (robot-at ?robot ?base))
                (at start (location-real ?bound))

                ;; Check for need-stereo so the planner only tries this action when the user
                ;; explicitly requests it.
                (at start (need-stereo ?robot ?order ?base ?bound))

                ;; Check collision avoidance
                (at start (location-available ?bound))
                (at start (locations-different ?check1 ?check2))
                (at start (move-connected ?check1 ?bound))
                (at start (move-connected ?check2 ?bound))
                (at start (location-available ?check1))
                (at start (location-available ?check2))
            )
        :effect
            (and
                ;; Grab and release robot mutex
                (at start (not (robot-available ?robot)))
                (at end (robot-available ?robot))

                ;; Update order
                (at end (assign (robot-order ?robot) (order-identity ?order)))

                ;; Grab and release reserved locations
                (at start (not (location-available ?bound)))
                (at end (location-available ?bound))

                ;; Update robot location - technically correct but not really needed
                ;(at start (not (robot-at ?robot ?base)))
                ;(at end (robot-at ?robot ?base))

                ;; Clear need-stereo so the planner won't try to use the stereo action
                ;; again after the user request is satisfied.
                (at end (not (need-stereo ?robot ?order ?base ?bound)))

                ;; Mark success
                (at end (completed-stereo ?robot ?order ?base ?bound))
            )
    )

    (:durative-action let-other-robot-reach
        :parameters (
            ?robot - robot
	    ?order - order
            ?other-loc - location  ;; location other robot needs to reach
	    ?other-robot - robot
        )
        :duration (= ?duration 0.001)  ;; VAL won't accept 0-duration actions
        :condition
            (and
                ;; Check robot mutex
                (at start (robot-available ?robot))

                ;; Check order
                (at start (< (robot-order ?robot) (order-identity ?order)))

		;; Check parameters make sense
		(at start (robots-different ?robot ?other-robot))

                ;; The main point is to wait until this condition is met
		(at start (robot-at ?other-robot ?other-loc))
            )
        :effect
            (and
                ;; Update order
                (at end (assign (robot-order ?robot) (order-identity ?order)))

		; Mark success
	        (at end (completed-let-other-robot-reach ?robot ?order ?other-loc))
            )
    )
)
