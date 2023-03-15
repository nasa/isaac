\page survey_manager Survey Manager

Control node for survey work crew participants, includes operator interface.

Task Life Cycle
---------
``
Unassigned Queue --(planning)--> Robot queue --(execution)–-> Verify Queue --(verified good)--> Complete
            /\                                      |             |
            |                                       |      (verified bad)
            |                                   (exception)       |
            |                                       |             |
       {New Task} <--(retry)-- Failed Queue  <-----<-------------<-
``
