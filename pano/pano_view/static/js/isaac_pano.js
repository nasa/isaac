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

/* Implement overview map logic for ISAAC panoramic tour. */

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
