/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

/**********************************************************************
 * Overview map for panoramic tour
 **********************************************************************/

function getDist(pos0, pos1) {
    return Math.sqrt((pos0[0] - pos1[0]) ** 2 + (pos0[1] - pos1[1]) ** 2);
}

function getClosestScene(pos, config, maxDistance) {
    var closestSceneName = null;
    var closestSceneDist = 99e+20;
    for (let [sceneName, scene] of Object.entries(config.scenes)) {
        var dist = getDist(pos, scene.overviewMapPosition);
        if (dist <= maxDistance && dist < closestSceneDist) {
            closestSceneName = sceneName;
            closestSceneDist = dist;
        }
    }
    return closestSceneName;
}

function getMousePos(event) {
    var mapOriginX = window.overviewMap.offsetLeft;
    var mapOriginY = window.overviewMap.offsetTop;
    var offsetX = event.clientX - mapOriginX;
    var offsetY = event.clientY - mapOriginY;
    return [offsetX, offsetY];
}

function overviewMapClick(event) {
    var sceneName = getClosestScene(getMousePos(event),
                                    window.initialConfig, 50);
    if (sceneName) {
        window.viewer.loadScene(sceneName);
    }
}

function overviewMapMouseLeave(event) {
    window.mapHighlight.style.display = "none";
}

function overviewMapMouseMove(event) {
    var sceneName = getClosestScene(getMousePos(event),
                                    window.initialConfig, 50);
    if (sceneName) {
        var pos = window.initialConfig.scenes[sceneName].overviewMapPosition;
        window.mapHighlight.style.left = pos[0] + "px";
        window.mapHighlight.style.top = pos[1] + "px";
        window.mapHighlight.style.display = "inline";
    } else {
        window.mapHighlight.style.display = "none";
    }
}

function updateMapCurrent() {
    var pos = window.viewer.getConfig().overviewMapPosition;
    var mapCurrent = window.mapCurrent;
    mapCurrent.style.left = pos[0] + "px";
    mapCurrent.style.top = pos[1] + "px";

    updateYaw();
}

function updateYaw() {
    var yaw = window.viewer.getConfig().yaw;
    var mapCurrent = window.mapCurrent;
    var northOffset = window.viewer.getConfig().northOffset || 0;
    mapCurrent.style.transform = (
        "translate(-50%, -50%) " +
            "rotate(" + (yaw + northOffset) + "deg) "
    );
}

function initIsaacPano(event) {
    var config = event.configFromURL;
    window.initialConfig = config;

    var uiContainer = document.getElementsByClassName('pnlm-ui')[0];

    var overviewMap = document.createElement('div');
    overviewMap.className = 'pnlm-overview-map pnlm-controls pnlm-control';
    uiContainer.appendChild(overviewMap);
    window.overviewMap = overviewMap;

    overviewMap.addEventListener('click', overviewMapClick);
    overviewMap.addEventListener('mousemove', overviewMapMouseMove);
    overviewMap.addEventListener('mouseleave', overviewMapMouseLeave);

    for (let [sceneName, scene] of Object.entries(config.scenes)) {
        var mapMarker = document.createElement('div');
        mapMarker.className = 'pnlm-map-marker';
        var pos = scene.overviewMapPosition;
        mapMarker.style.left = pos[0] + "px";
        mapMarker.style.top = pos[1] + "px";
        overviewMap.appendChild(mapMarker);
    }

    var mapHighlight = document.createElement('div');
    mapHighlight.className = 'pnlm-map-highlight';
    overviewMap.appendChild(mapHighlight);
    window.mapHighlight = mapHighlight;

    var mapCurrent = document.createElement('div');
    mapCurrent.className = 'pnlm-map-current';
    overviewMap.appendChild(mapCurrent);
    window.mapCurrent = mapCurrent;

    updateMapCurrent();

    window.viewer.on("scenechange", updateMapCurrent);
    window.viewer.on("mouseup", updateYaw);
    window.viewer.on("touchend", updateYaw);
}

document.addEventListener('pannellumloaded', initIsaacPano, false);

/**********************************************************************
 * View options dropdown menu
 **********************************************************************/

function isaacSleep(secs) {
    return new Promise(resolve => setTimeout(resolve, secs * 1000));
}

/* When the user clicks on the button, toggle between hiding and showing the dropdown content */
function isaacToggleDropDown() {
    document.getElementById("isaac-view-dropdown").classList.toggle("show");
}

function isaacSetVisibility(className, visibility) {
    var elts = document.getElementsByClassName(className)
    var newDisplayStyle = visibility ? 'block' : 'none';
    for (const elt of elts) {
	elt.style.display = newDisplayStyle;
    }
}

function isaacShowNavControls(visibility) {
    isaacSetVisibility('pnlm-controls-container', visibility);
}

function isaacShowOverviewMap(visibility) {
    isaacSetVisibility('pnlm-overview-map', visibility);
}

function isaacShowHotSpotType(hotSpotType, visibility) {
    var config = window.viewer.getConfig();
    var currentSceneId = window.viewer.getScene();

    // Update hotSpots for all scenes. That way the visibility change
    // will persist when the scene changes.
    for (let [sceneId, scene] of Object.entries(config.scenes)) {
	// back up original complete hotSpots array if needed
	if (!scene.hasOwnProperty('initialHotSpots')) {
	    scene.initialHotSpots = [...scene.hotSpots];
	}

	if (visibility) {
	    // add hotSpots matching type
	    for (const hotSpot of scene.initialHotSpots) {
		if (hotSpot.type == hotSpotType) {
		    // console.log("window.viewer.addHotSpot(" + hotSpot.id + "," + sceneId + ");");
		    window.viewer.addHotSpot(hotSpot, sceneId);
		}
	    }
	} else {
	    // remove hotSpots matching type
	    var hotSpotsCopy = [...scene.hotSpots];
	    for (const hotSpot of hotSpotsCopy) {
		if (hotSpot.type == hotSpotType) {
		    // console.log("window.viewer.removeHotSpot(" + hotSpot.id + "," + sceneId + ");");
		    window.viewer.removeHotSpot(hotSpot.id, sceneId);
		}
	    }
	}
    }
}

function isaacShowSceneLinks(visibility) {
    isaacShowHotSpotType("scene", visibility);
}

function isaacShowSourceImageLinks(visibility) {
    isaacShowHotSpotType("info", visibility);
}

const ISAAC_CHANGE_VISIBILITY_HANDLERS = {
    "isaac-show-nav-controls": isaacShowNavControls,
    "isaac-show-overview-map": isaacShowOverviewMap,
    "isaac-show-scene-links": isaacShowSceneLinks,
    "isaac-show-source-image-links": isaacShowSourceImageLinks,
};

function isaacToggleEntryCheckBox(event) {
    var elt = event.srcElement;
    elt.classList.toggle("checked");
    var visibility = elt.classList.contains("checked");
    ISAAC_CHANGE_VISIBILITY_HANDLERS[elt.id](visibility);
}

function isaacInitViewDropDown() {
    document.getElementsByClassName("isaac-drop-button")[0].onclick = isaacToggleDropDown;

    var toggleEntries = document.getElementsByClassName("isaac-toggle-entry");
    for (const entry of toggleEntries) {
	entry.onclick = isaacToggleEntryCheckBox;
    }

    // Close the dropdown if the user clicks outside of it
    window.onclick = async function(event) {
	if (!event.target.matches('.isaac-drop-button')) {
	    var dropdowns = document.getElementsByClassName("isaac-dropdown-content");
	    for (const dropdown of dropdowns) {
		if (dropdown.classList.contains('show')) {
		    await isaacSleep(0.4);
		    dropdown.classList.remove('show');
		}
	    }
	}
    }
}

isaacInitViewDropDown();
