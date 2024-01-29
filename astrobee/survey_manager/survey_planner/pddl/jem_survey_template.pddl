{{ header }}
(define (problem jem-survey)
    (:domain survey-manager)
    (:metric minimize (total-time))
    (:objects
        {{ objects }}
    )

    (:goal
        (and
            {{ goals }}
        )
    )

    (:init
        ;; === Static predicates ===
        {{ static_predicates }}

        ;; === Dynamic predicates ===
        {{ dynamic_predicates }}

        ;; === Static numeric fluents ===
        {{ static_fluents }}

        ;; === Dynamic numeric fluents ===
        {{ dynamic_fluents }}
    )  ;; end :init
)  ;; end problem
