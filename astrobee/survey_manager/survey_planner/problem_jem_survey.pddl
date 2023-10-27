(define (problem jem-survey)
    (:domain survey-manager)
    (:metric minimize (total-time))
    (:objects
        ;; Note: bay0 and bay8 are bogus locations. See location-real docs.
        bay0 bay1 bay2 bay3 bay4 bay5 bay6 bay7 bay8 berth1 berth2 - location
        bumble honey - robot
        o0 o1 o2 o3 o4 o5 o6 o7 o8 o9 - order
        run1 run2 run3 run4 run5 - run-number
    )

    (:goal
        (and
            ;; Bumble
            (completed-panorama bumble o0 bay4 run1)
            (completed-panorama bumble o1 bay3 run1)
            (completed-panorama bumble o2 bay2 run1)
            (completed-panorama bumble o3 bay1 run1)
            (completed-stereo   bumble o4 bay1 bay4 run1)

            ;; We want Bumble to return to its berth at the end of the run, but adding this goal
            ;; causes POPF to get confused and greatly increase the total run time. For some reason,
            ;; it doesn't notice it can use the same plan as without this goal and then add some
            ;; motion actions at the end to achieve this goal. Instead, it falls back to only
            ;; undocking one robot at a time, which slows things down by about 2x.
            ;(robot-at bumble berth1)  ;; return to berth when done

            ;; Honey
            (completed-panorama honey o0 bay7 run1)
            (completed-panorama honey o1 bay6 run1)
            (completed-panorama honey o2 bay5 run1)
            (completed-stereo   honey o3 bay4 bay7 run1)
            (robot-at honey berth2)  ;; return to berth when done
        )
    )

    (:init
        ;; === Static predicates ===
        (move-connected bay0 bay1)
        (move-connected bay1 bay0)
        (move-connected bay1 bay2)
        (move-connected bay2 bay1)
        (move-connected bay2 bay3)
        (move-connected bay3 bay2)
        (move-connected bay3 bay4)
        (move-connected bay4 bay3)
        (move-connected bay4 bay5)
        (move-connected bay5 bay4)
        (move-connected bay5 bay6)
        (move-connected bay6 bay5)
        (move-connected bay6 bay7)
        (move-connected bay7 bay6)
        (move-connected bay7 bay8)
        (move-connected bay8 bay7)

        (location-real bay1)
        (location-real bay2)
        (location-real bay3)
        (location-real bay4)
        (location-real bay5)
        (location-real bay6)
        (location-real bay7)

        (dock-connected bay7 berth1)
        (dock-connected bay7 berth2)

        (robots-different bumble honey)
        (robots-different honey bumble)

        (locations-different bay0 bay1)
        (locations-different bay0 bay2)
        (locations-different bay0 bay3)
        (locations-different bay0 bay4)
        (locations-different bay0 bay5)
        (locations-different bay0 bay6)
        (locations-different bay0 bay7)
        (locations-different bay0 bay8)
        (locations-different bay1 bay0)
        (locations-different bay1 bay2)
        (locations-different bay1 bay3)
        (locations-different bay1 bay4)
        (locations-different bay1 bay5)
        (locations-different bay1 bay6)
        (locations-different bay1 bay7)
        (locations-different bay1 bay8)
        (locations-different bay2 bay0)
        (locations-different bay2 bay1)
        (locations-different bay2 bay3)
        (locations-different bay2 bay4)
        (locations-different bay2 bay5)
        (locations-different bay2 bay6)
        (locations-different bay2 bay7)
        (locations-different bay2 bay8)
        (locations-different bay3 bay0)
        (locations-different bay3 bay1)
        (locations-different bay3 bay2)
        (locations-different bay3 bay4)
        (locations-different bay3 bay5)
        (locations-different bay3 bay6)
        (locations-different bay3 bay7)
        (locations-different bay3 bay8)
        (locations-different bay4 bay0)
        (locations-different bay4 bay1)
        (locations-different bay4 bay2)
        (locations-different bay4 bay3)
        (locations-different bay4 bay5)
        (locations-different bay4 bay6)
        (locations-different bay4 bay7)
        (locations-different bay4 bay8)
        (locations-different bay5 bay0)
        (locations-different bay5 bay1)
        (locations-different bay5 bay2)
        (locations-different bay5 bay3)
        (locations-different bay5 bay4)
        (locations-different bay5 bay6)
        (locations-different bay5 bay7)
        (locations-different bay5 bay8)
        (locations-different bay6 bay0)
        (locations-different bay6 bay1)
        (locations-different bay6 bay2)
        (locations-different bay6 bay3)
        (locations-different bay6 bay4)
        (locations-different bay6 bay5)
        (locations-different bay6 bay7)
        (locations-different bay6 bay8)
        (locations-different bay7 bay0)
        (locations-different bay7 bay1)
        (locations-different bay7 bay2)
        (locations-different bay7 bay3)
        (locations-different bay7 bay4)
        (locations-different bay7 bay5)
        (locations-different bay7 bay6)
        (locations-different bay7 bay8)
        (locations-different bay8 bay0)
        (locations-different bay8 bay1)
        (locations-different bay8 bay2)
        (locations-different bay8 bay3)
        (locations-different bay8 bay4)
        (locations-different bay8 bay5)
        (locations-different bay8 bay6)
        (locations-different bay8 bay7)

        ;; === Dynamic predicates ===
        (robot-available bumble)
        (robot-available honey)

        (robot-at bumble berth1)
        (robot-at honey berth2)

        (location-available bay8)
        (location-available bay7)
        (location-available bay6)
        (location-available bay5)
        (location-available bay4)
        (location-available bay3)
        (location-available bay2)
        (location-available bay1)
        (location-available bay0)

        ;; These need-stereo predicates must be asserted with identical parameters to the
        ;; stereo-completed goals.  See the need-stereo docs for more.
        (need-stereo   bumble o4 bay1 bay4 run1)
        (need-stereo   honey o3 bay4 bay7 run1)

        ;; === Static numeric fluents ===
        (= (order-identity o0) 0)
        (= (order-identity o1) 1)
        (= (order-identity o2) 2)
        (= (order-identity o3) 3)
        (= (order-identity o4) 4)
        (= (order-identity o5) 5)
        (= (order-identity o6) 6)
        (= (order-identity o7) 7)
        (= (order-identity o8) 8)
        (= (order-identity o9) 9)

        ;; === Dynamic numeric fluents ===
        (= (robot-order bumble) -1)
        (= (robot-order honey) -1)
    )  ;; end :init
)  ;; end problem
