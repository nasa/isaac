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
    var mapRect = window.overviewMap.getBoundingClientRect();
    var offsetX = event.clientX - mapRect.x;
    var offsetY = event.clientY - mapRect.y;
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
    var yaw = window.viewer.getYaw();
    var mapCurrent = window.mapCurrent;
    var northOffset = window.viewer.getConfig().northOffset || 0;
    mapCurrent.style.transform = (
        "translate(-50%, -50%) " +
            "rotate(" + (yaw + northOffset) + "deg) "
    );
}

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
    isaacSetVisibility('pnlm-panorama-info', visibility);
}

function isaacShowOverviewMap(visibility) {
    isaacSetVisibility('pnlm-overview-map', visibility);
}

/* Return true if `hotSpot` in `sceneId` has annotations according to
 * `storageItem`.
 *
 * @param storageItem The JSON-parsed HTML5 local storage object containing annotations.
 * @param sceneId The sceneId for the current pano scene from the tour config.
 * @param hotSpot A source image hotspot object from the tour config.
 */
function isaacHasAnnotations(storageItem, sceneId, hotSpot) {
    const sceneAnnotations = storageItem[sceneId];
    if (sceneAnnotations == undefined) return false;

    const imageAnnotations = sceneAnnotations[hotSpot.id];
    if (imageAnnotations == undefined) return false;

    return imageAnnotations.length > 0;
}

/* Set the cssClass property of `hotSpot` so that it is rendered in the
 * annotated style or not depending on the `isAnnotated` flag.
 *
 * @param hotSpot A source image hotspot object from the tour config.
 * @param isAnnotated If true, style the hotspot in the annotated style.
 */
function isaacSetAnnotatedStyle(hotSpot, isAnnotated) {
    const unannotatedCssClass = hotSpot.cssClass.replace("isaac-annotated", "");
    const annotateClass = "isaac-annotated";
    let cssClass = unannotatedCssClass;
    if (isAnnotated) {
	cssClass += " " + annotateClass;
    }
    hotSpot.cssClass = cssClass;
}

function isaacIsHotSpotVisible(hotSpot) {
    const viewConfig = window.hotSpotViewConfig;
    if (hotSpot.type == "scene") {
	return viewConfig.showSceneLinks;
    } else if (hotSpot.type == "info") {
	return viewConfig.showSourceImageLinks;
    } else {
	console.log("ERROR: Unknown hotspot type " + hotSpot.type);
	return false;
    }
}

function isaacRefreshHotSpots() {
    var config = window.viewer.getConfig();
    var storageItem = isaacStorageGetRoot();

    for (let [sceneId, scene] of Object.entries(config.scenes)) {
	// Remove all hotspots in the current Pannellum view. Note: We
	// have to copy the current hotspots first to avoid iterating
	// through an array while modifying it.
	const currentHotSpots = [...scene.hotSpots];
	for (let hotSpot of currentHotSpots) {
	    window.viewer.removeHotSpot(hotSpot.id, sceneId);
	}

	// Add back hotspots that should be visible currently,
	// adjusting styling if needed.
	for (let hotSpot of scene.initialHotSpots) {
	    if (isaacIsHotSpotVisible(hotSpot)) {
		if (hotSpot.type == "info") {
		    isaacSetAnnotatedStyle(hotSpot, isaacHasAnnotations(storageItem, sceneId, hotSpot));
		}
		window.viewer.addHotSpot(hotSpot, sceneId);
	    }
	}
    }
}

function isaacShowSceneLinks(visibility) {
    window.hotSpotViewConfig.showSceneLinks = visibility;
    isaacRefreshHotSpots();
}

function isaacShowSourceImageLinks(visibility) {
    window.hotSpotViewConfig.showSourceImageLinks = visibility;
    isaacRefreshHotSpots();
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

function isaacDeepCopy(obj) {
    return JSON.parse(JSON.stringify(obj));
}

function isaacInitViewDropDown() {
    document.getElementsByClassName("isaac-drop-button")[0].onclick = isaacToggleDropDown;

    var toggleEntries = document.getElementsByClassName("isaac-toggle-entry");
    for (const entry of toggleEntries) {
	entry.onclick = isaacToggleEntryCheckBox;
    }

    // Back up the initial/complete list of hotspots for each scene,
    // given we may need to remove and add them back later based on the
    // user's selected view options.
    var config = window.viewer.getConfig();
    for (let [sceneId, scene] of Object.entries(config.scenes)) {
	scene.initialHotSpots = [...scene.hotSpots];
    }

    window.hotSpotViewConfig = {
	showSceneLinks: true,
	showSourceImageLinks: true
    };

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

/* Return an array of [sceneId, imageid] for images that have annotations. */
function isaacGetImagesWithTargets() {
    var storageItem = isaacStorageGetRoot();
    var result = [];
    for (const [sceneId, imageAnnotationsMap] of Object.entries(storageItem)) {
	for (const [imageId, annotations] of Object.entries(imageAnnotationsMap)) {
	    if (annotations.length > 0) {
		result.push([sceneId, imageId]);
	    }
	};
    };
    return result;
}

/* Process an annotation update that comes from the annotation storage
 * object. This highlights source images that contain targets and enables
 * the target selection Next/Previous buttons.
 */
function isaacPanoProcessStorageUpdate() {
    isaacRefreshHotSpots();
}

/* Change the annotation review index by `delta`. Each review index
 * refers to a sceneId, imageId pair. Changing the index switches to
 * the scene and pans to the image.
 *
 * @param delta Specify delta = 1 for the Next button or -1 for the Previous button.
 */
function isaacPanoReviewUpdate(delta) {
    const reviewImages = isaacGetImagesWithTargets();
    if (reviewImages.length == 0) return;
    ISAAC_REVIEW_INDEX = (ISAAC_REVIEW_INDEX + delta + reviewImages.length) % reviewImages.length;
    const [sceneId, imageId] = reviewImages[ISAAC_REVIEW_INDEX];
    const hotSpot = window.initialConfig.scenes[sceneId].hotSpots.find((hs) => (hs.id == imageId));
    if (hotSpot == undefined) {
	console.log("isaacPanoReviewUpdate: invalid sceneId, imageId:", sceneId, imageId);
	return;
    }
    const reviewHfov = 50.0;
    if (window.viewer.getScene() != sceneId) {
	window.viewer.loadScene(sceneId, hotSpot.pitch, hotSpot.yaw, reviewHfov);
    } else {
	window.viewer.lookAt(hotSpot.pitch, hotSpot.yaw, reviewHfov);
    }
}

/* Download the full pano image as a file called '<sceneId>_pano.jpg'. */
function isaacDownloadPanoImage() {
    const sceneId = window.viewer.getScene();
    const a = document.createElement('a');
    a.href = "scenes/" + sceneId + "/pano.jpg";
    a.download = sceneId + "_pano.jpg";
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
}

/**********************************************************************
 * Main initialization
 **********************************************************************/

function isaacPanoInit(event) {
    var config = event.configFromURL;

    if (config.initialAnnotations == null) {
        config.initialAnnotations = {};
    }
    // Initialize default annotation content in local storage, making
    // it available to source image tabs that don't load the config
    // from tour.json, just in case they need it. (This seems
    // unlikely.)
    window.localStorage.setItem(ISAAC_DEFAULT_CONTENT_KEY, JSON.stringify(config.initialAnnotations));

    // Initialize annotations to the default if none are present.
    var storageItem = isaacStorageGetRoot();
    delete storageItem.source_window_id; // ignore this field if present
    if (Object.keys(storageItem).length == 0) {
        isaacStorageSetRoot(config.initialAnnotations);
    }

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

    // Pannellum's default HFOV bounds of [50, 120] don't allow the
    // user to zoom in extremely close, which is sometimes useful.
    window.viewer.setHfovBounds([5, 120]);

    window.viewer.on("scenechange", updateMapCurrent);
    window.viewer.on("mouseup", updateYaw);
    window.viewer.on("touchend", updateYaw);
    window.viewer.on("animatefinished", updateYaw);

    isaacInitViewDropDown();
    isaacConfigureLoadSaveClear(isaacPanoProcessStorageUpdate);

    document.getElementById('isaac-pano-image').addEventListener('click', isaacDownloadPanoImage);

    document.getElementById('isaac-previous').addEventListener('click', event => isaacPanoReviewUpdate(-1));
    document.getElementById('isaac-next').addEventListener('click', event => isaacPanoReviewUpdate(1));

    // Load annotations
    isaacPanoProcessStorageUpdate();

    // Handle annotation changes triggered by other windows
    window.addEventListener(
        "storage",
        (event) => isaacPanoProcessStorageUpdate()
    );
}

document.addEventListener('pannellumloaded', isaacPanoInit, false);
