\page action_testing Action Testing

## Simulation

### Move

    set instance bumble robot
    set instance jem_bay7 location
    set instance jem_bay6 location
    set instance jem_bay5 location

    set predicate (robot-available bumble)
    set predicate (robot-at bumble jem_bay7)
    set predicate (location-available jem_bay6)

    set predicate (move-connected jem_bay7 jem_bay6)
    set predicate (location-real jem_bay6)
    set predicate (locations-different jem_bay5 jem_bay7)
    set predicate (move-connected jem_bay5 jem_bay6)
    set predicate (location-available jem_bay5)


    run action (move bumble jem_bay7 jem_bay6 jem_bay5)

### Dock

    set instance bumble robot
    set instance jem_bay7 location
    set instance berth1 location

    set predicate (robot-available bumble)
    set predicate (robot-at bumble jem_bay7)
    set predicate (dock-connected jem_bay7 berth1)
    set predicate (location-available berth1)

    run action (dock bumble jem_bay7 berth1)


### Undock


    set instance bumble robot
    set instance jem_bay6 location
    set instance jem_bay7 location
    set instance jem_bay8 location
    set instance berth1 location

    set predicate (robot-available bumble)
    set predicate (robot-at bumble berth1)
    set predicate (dock-connected jem_bay7 berth1)
    set predicate (location-available jem_bay7)
    set predicate (location-real jem_bay7)
    set predicate (locations-different jem_bay8 jem_bay6)
    set predicate (move-connected jem_bay8 jem_bay7)
    set predicate (move-connected jem_bay6 jem_bay7)
    set predicate (location-available jem_bay8)
    set predicate (location-available jem_bay6)

    run action (undock bumble berth1 jem_bay7 jem_bay8 jem_bay6)


### Panorama

    set instance bumble robot
    set instance jem_bay6 location
    set instance  o0 order

    set predicate (robot-available bumble)

    set function (= (order-identity o0) 0)
    set function (= (robot-order bumble) -1)

    set predicate (robot-at bumble jem_bay6)

    run action (panorama bumble o0 jem_bay6)


### Stereo


    set instance bumble robot
    set instance jem_bay7 location
    set instance jem_bay6 location
    set instance jem_bay5 location
    set instance jem_bay4 location
    set instance jem_bay3 location
    set instance  o0 order


    set predicate (robot-available bumble)
    set predicate (robot-at bumble jem_bay7)
    set predicate (location-real jem_bay4)
    set predicate (need-stereo bumble o0 jem_bay7 jem_bay4)
    set predicate (location-available jem_bay4)
    set predicate (locations-different jem_bay5 jem_bay3)
    set predicate (move-connected jem_bay5 jem_bay4)
    set predicate (move-connected jem_bay3 jem_bay4)
    set predicate (location-available jem_bay5)
    set predicate (location-available jem_bay3)

    set function (= (order-identity o0) 0)
    set function (= (robot-order bumble) -1)


    run action (stereo bumble o0 jem_bay7 jem_bay4 jem_bay5 jem_bay3)

## Granite table

### Move

    set instance wannabee robot
    set instance gra_p1 location
    set instance gra_p2 location
    set instance jem_bay5 location

    set predicate (robot-available wannabee)
    set predicate (robot-at wannabee gra_p1)
    set predicate (location-available gra_p2)

    set predicate (move-connected gra_p1 gra_p2)
    set predicate (location-real gra_p2)
    set predicate (locations-different jem_bay5 gra_p1)
    set predicate (move-connected jem_bay5 gra_p2)
    set predicate (location-available jem_bay5)


    run action (move wannabee gra_p1 gra_p2 jem_bay5)

### Dock

    set instance wannabee robot
    set instance gra_p1 location
    set instance berth2 location

    set predicate (robot-available wannabee)
    set predicate (robot-at wannabee gra_p1)
    set predicate (dock-connected gra_p1 berth2)
    set predicate (location-available berth2)

    run action (dock wannabee gra_p1 berth2)


### Undock


    set instance wannabee robot
    set instance gra_p2 location
    set instance gra_p1 location
    set instance gra_p0 location
    set instance berth1 location

    set predicate (robot-available wannabee)
    set predicate (robot-at wannabee berth1)
    set predicate (dock-connected gra_p1 berth1)
    set predicate (location-available gra_p1)
    set predicate (location-real gra_p1)
    set predicate (locations-different gra_p0 gra_p2)
    set predicate (move-connected gra_p0 gra_p1)
    set predicate (move-connected gra_p2 gra_p1)
    set predicate (location-available gra_p0)
    set predicate (location-available gra_p2)

    run action (undock wannabee berth1 gra_p1 gra_p0 gra_p2)


### Panorama

    set instance wannabee robot
    set instance gra_p2 location
    set instance  o0 order

    set predicate (robot-available wannabee)

    set function (= (order-identity o0) 0)
    set function (= (robot-order wannabee) -1)

    set predicate (robot-at wannabee gra_p2)

    run action (panorama wannabee o0 gra_p2)


### Stereo


    set instance wannabee robot
    set instance gra_p1 location
    set instance gra_p2 location
    set instance jem_bay5 location
    set instance jem_bay4 location
    set instance jem_bay3 location
    set instance  o0 order


    set predicate (robot-available wannabee)
    set predicate (robot-at wannabee gra_p1)
    set predicate (location-real jem_bay4)
    set predicate (need-stereo wannabee o0 gra_p1 jem_bay4)
    set predicate (location-available jem_bay4)
    set predicate (locations-different jem_bay5 jem_bay3)
    set predicate (move-connected jem_bay5 jem_bay4)
    set predicate (move-connected jem_bay3 jem_bay4)
    set predicate (location-available jem_bay5)
    set predicate (location-available jem_bay3)

    set function (= (order-identity o0) 0)
    set function (= (robot-order wannabee) -1)


    run action (stereo wannabee o0 gra_p1 jem_bay4 jem_bay5 jem_bay3)