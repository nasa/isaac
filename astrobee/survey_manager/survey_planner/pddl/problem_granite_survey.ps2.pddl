set instance gra_bay1 location
set instance gra_bay2 location
set instance gra_bay3 location
set instance jem_bay0 location
set instance jem_bay8 location
set instance berth1 location
set instance berth2 location
set instance bsharp robot
set instance wannabee robot
set instance o0 order
set instance o1 order
set instance o2 order
set instance o3 order
set instance o4 order
set goal (and (completed-panorama bsharp o0 gra_bay1) (completed-stereo bsharp o1 gra_bay1 gra_bay1) (robot-at bsharp berth1) (completed-panorama wannabee o1 gra_bay3) (completed-stereo wannabee o4 gra_bay3 gra_bay3) (robot-at wannabee berth2))
set predicate (move-connected gra_bay1 gra_bay2)
set predicate (move-connected gra_bay2 gra_bay1)
set predicate (move-connected gra_bay2 gra_bay3)
set predicate (move-connected gra_bay3 gra_bay2)
set predicate (move-connected gra_bay3 jem_bay0)
set predicate (move-connected jem_bay0 gra_bay3)
set predicate (move-connected jem_bay0 jem_bay8)
set predicate (move-connected jem_bay8 jem_bay0)
set predicate (location-real gra_bay1)
set predicate (location-real gra_bay2)
set predicate (location-real gra_bay3)
set predicate (dock-connected gra_bay1 berth1)
set predicate (dock-connected gra_bay3 berth2)
set predicate (robots-different bsharp wannabee)
set predicate (robots-different wannabee bsharp)
set predicate (locations-different gra_bay1 gra_bay2)
set predicate (locations-different gra_bay1 gra_bay3)
set predicate (locations-different gra_bay1 jem_bay0)
set predicate (locations-different gra_bay1 jem_bay8)
set predicate (locations-different gra_bay2 gra_bay1)
set predicate (locations-different gra_bay2 gra_bay3)
set predicate (locations-different gra_bay2 jem_bay0)
set predicate (locations-different gra_bay2 jem_bay8)
set predicate (locations-different gra_bay3 gra_bay1)
set predicate (locations-different gra_bay3 gra_bay2)
set predicate (locations-different gra_bay3 jem_bay0)
set predicate (locations-different gra_bay3 jem_bay8)
set predicate (locations-different jem_bay0 gra_bay1)
set predicate (locations-different jem_bay0 gra_bay2)
set predicate (locations-different jem_bay0 gra_bay3)
set predicate (locations-different jem_bay0 jem_bay8)
set predicate (locations-different jem_bay8 gra_bay1)
set predicate (locations-different jem_bay8 gra_bay2)
set predicate (locations-different jem_bay8 gra_bay3)
set predicate (locations-different jem_bay8 jem_bay0)
set predicate (robot-available bsharp)
set predicate (robot-available wannabee)
set predicate (robot-at bsharp berth1)
set predicate (robot-at wannabee berth2)
set predicate (location-available gra_bay1)
set predicate (location-available gra_bay2)
set predicate (location-available gra_bay3)
set predicate (location-available jem_bay0)
set predicate (location-available jem_bay8)
set predicate (need-stereo bsharp o1 gra_bay1 gra_bay1)
set predicate (need-stereo wannabee o4 gra_bay3 gra_bay3)
set function (= (order-identity o0) 0)
set function (= (order-identity o1) 1)
set function (= (order-identity o2) 2)
set function (= (order-identity o3) 3)
set function (= (order-identity o4) 4)
set function (= (robot-order bsharp) -1)
set function (= (robot-order wannabee) -1)
