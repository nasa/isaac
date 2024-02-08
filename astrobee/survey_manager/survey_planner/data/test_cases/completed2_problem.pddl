;; Auto-generated by problem_generator.py. Do not edit!
;; Command was: src/survey_planner/problem_generator.py --config=data/survey_static.yaml,data/test_cases/completed2_config.yaml --output=data/test_cases/completed2_problem.pddl
;; Working directory was: data/test_cases
;; Problem template: ../../pddl/jem_survey_template.pddl
;; Config 1: data/survey_static.yaml
;; Config 2: data/test_cases/completed2_config.yaml

(define (problem jem-survey)
    (:domain survey-manager)
    (:metric minimize (total-time))
    (:objects
        jem_bay0 jem_bay1 jem_bay2 jem_bay3 jem_bay4 jem_bay5 jem_bay6 jem_bay7 jem_bay8 berth1 berth2 - location
        bumble honey - robot
        o0 o1 o2 o3 o4 - order
    )

    (:goal
        (and
            (completed-panorama bumble o0 jem_bay4)
            (completed-panorama bumble o1 jem_bay3)
            (completed-panorama bumble o2 jem_bay2)
            (completed-panorama bumble o3 jem_bay1)
            (completed-stereo bumble o4 jem_bay1 jem_bay4)
            (robot-at bumble berth1)
            (completed-panorama honey o1 jem_bay7)
            (completed-panorama honey o2 jem_bay6)
            (completed-panorama honey o3 jem_bay5)
            (completed-stereo honey o4 jem_bay7 jem_bay4)
            (robot-at honey berth2)
        )
    )

    (:init
        ;; === Static predicates ===
        (move-connected jem_bay0 jem_bay1)
        (move-connected jem_bay1 jem_bay0)
        (move-connected jem_bay1 jem_bay2)
        (move-connected jem_bay2 jem_bay1)
        (move-connected jem_bay2 jem_bay3)
        (move-connected jem_bay3 jem_bay2)
        (move-connected jem_bay3 jem_bay4)
        (move-connected jem_bay4 jem_bay3)
        (move-connected jem_bay4 jem_bay5)
        (move-connected jem_bay5 jem_bay4)
        (move-connected jem_bay5 jem_bay6)
        (move-connected jem_bay6 jem_bay5)
        (move-connected jem_bay6 jem_bay7)
        (move-connected jem_bay7 jem_bay6)
        (move-connected jem_bay7 jem_bay8)
        (move-connected jem_bay8 jem_bay7)
        (location-real jem_bay1)
        (location-real jem_bay2)
        (location-real jem_bay3)
        (location-real jem_bay4)
        (location-real jem_bay5)
        (location-real jem_bay6)
        (location-real jem_bay7)
        (dock-connected jem_bay7 berth1)
        (dock-connected jem_bay7 berth2)
        (robots-different bumble honey)
        (robots-different honey bumble)
        (locations-different jem_bay0 jem_bay1)
        (locations-different jem_bay0 jem_bay2)
        (locations-different jem_bay0 jem_bay3)
        (locations-different jem_bay0 jem_bay4)
        (locations-different jem_bay0 jem_bay5)
        (locations-different jem_bay0 jem_bay6)
        (locations-different jem_bay0 jem_bay7)
        (locations-different jem_bay0 jem_bay8)
        (locations-different jem_bay1 jem_bay0)
        (locations-different jem_bay1 jem_bay2)
        (locations-different jem_bay1 jem_bay3)
        (locations-different jem_bay1 jem_bay4)
        (locations-different jem_bay1 jem_bay5)
        (locations-different jem_bay1 jem_bay6)
        (locations-different jem_bay1 jem_bay7)
        (locations-different jem_bay1 jem_bay8)
        (locations-different jem_bay2 jem_bay0)
        (locations-different jem_bay2 jem_bay1)
        (locations-different jem_bay2 jem_bay3)
        (locations-different jem_bay2 jem_bay4)
        (locations-different jem_bay2 jem_bay5)
        (locations-different jem_bay2 jem_bay6)
        (locations-different jem_bay2 jem_bay7)
        (locations-different jem_bay2 jem_bay8)
        (locations-different jem_bay3 jem_bay0)
        (locations-different jem_bay3 jem_bay1)
        (locations-different jem_bay3 jem_bay2)
        (locations-different jem_bay3 jem_bay4)
        (locations-different jem_bay3 jem_bay5)
        (locations-different jem_bay3 jem_bay6)
        (locations-different jem_bay3 jem_bay7)
        (locations-different jem_bay3 jem_bay8)
        (locations-different jem_bay4 jem_bay0)
        (locations-different jem_bay4 jem_bay1)
        (locations-different jem_bay4 jem_bay2)
        (locations-different jem_bay4 jem_bay3)
        (locations-different jem_bay4 jem_bay5)
        (locations-different jem_bay4 jem_bay6)
        (locations-different jem_bay4 jem_bay7)
        (locations-different jem_bay4 jem_bay8)
        (locations-different jem_bay5 jem_bay0)
        (locations-different jem_bay5 jem_bay1)
        (locations-different jem_bay5 jem_bay2)
        (locations-different jem_bay5 jem_bay3)
        (locations-different jem_bay5 jem_bay4)
        (locations-different jem_bay5 jem_bay6)
        (locations-different jem_bay5 jem_bay7)
        (locations-different jem_bay5 jem_bay8)
        (locations-different jem_bay6 jem_bay0)
        (locations-different jem_bay6 jem_bay1)
        (locations-different jem_bay6 jem_bay2)
        (locations-different jem_bay6 jem_bay3)
        (locations-different jem_bay6 jem_bay4)
        (locations-different jem_bay6 jem_bay5)
        (locations-different jem_bay6 jem_bay7)
        (locations-different jem_bay6 jem_bay8)
        (locations-different jem_bay7 jem_bay0)
        (locations-different jem_bay7 jem_bay1)
        (locations-different jem_bay7 jem_bay2)
        (locations-different jem_bay7 jem_bay3)
        (locations-different jem_bay7 jem_bay4)
        (locations-different jem_bay7 jem_bay5)
        (locations-different jem_bay7 jem_bay6)
        (locations-different jem_bay7 jem_bay8)
        (locations-different jem_bay8 jem_bay0)
        (locations-different jem_bay8 jem_bay1)
        (locations-different jem_bay8 jem_bay2)
        (locations-different jem_bay8 jem_bay3)
        (locations-different jem_bay8 jem_bay4)
        (locations-different jem_bay8 jem_bay5)
        (locations-different jem_bay8 jem_bay6)
        (locations-different jem_bay8 jem_bay7)

        ;; === Dynamic predicates ===
        (robot-available bumble)
        (robot-available honey)
        (robot-at bumble jem_bay3)
        (robot-at honey jem_bay6)
        (location-available berth1)
        (location-available berth2)
        (location-available jem_bay0)
        (location-available jem_bay1)
        (location-available jem_bay2)
        (location-available jem_bay4)
        (location-available jem_bay5)
        (location-available jem_bay7)
        (location-available jem_bay8)
        (completed-panorama bumble o0 jem_bay4)
        (completed-panorama bumble o1 jem_bay3)
        (completed-panorama honey o1 jem_bay7)
        (completed-panorama honey o2 jem_bay6)
        (need-stereo bumble o4 jem_bay1 jem_bay4)
        (need-stereo honey o4 jem_bay7 jem_bay4)

        ;; === Static numeric fluents ===
        (= (order-identity o0) 0)
        (= (order-identity o1) 1)
        (= (order-identity o2) 2)
        (= (order-identity o3) 3)
        (= (order-identity o4) 4)

        ;; === Dynamic numeric fluents ===
        (= (robot-order bumble) -1)
        (= (robot-order honey) -1)
    )  ;; end :init
)  ;; end problem
