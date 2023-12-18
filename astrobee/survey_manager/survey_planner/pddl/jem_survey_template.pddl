{{ header }}
(define (problem jem-survey)
    (:domain survey-manager)
    (:metric minimize (total-time))
    (:objects
        {{ bays }} {{ berths }} - location
        {{ robots }} - robot
        {{ orders }} - order
        run1 run2 run3 run4 run5 - run-number
    )

    (:goal
        (and
            {{ goals }}
        )
    )

    (:init
        ;; === Static predicates ===
        {{ move_connected_predicates }}

        {{ location_real_predicates }}

        {{ dock_connected_predicates }}

        {{ robots_different_predicates }}

        {{ locations_different_predicates }}

        ;; === Dynamic predicates ===
        {{ robot_available_predicates }}

        {{ robot_at_predicates }}

        {{ location_available_predicates }}

        ;; need-stereo predicates must be asserted with identical parameters to the
        ;; stereo-completed goals.  See the need-stereo docs for more.
        {{ need_stereo_predicates }}

        ;; === Static numeric fluents ===
        {{ order_identity_fluents }}

        ;; === Dynamic numeric fluents ===
        {{ robot_order_fluents }}
    )  ;; end :init
)  ;; end problem

;; Include raw high-level config in problem in case an (ISAAC-custom) planner prefers to use it.

;; BEGIN CONFIG
{{ config }}
;; END CONFIG
