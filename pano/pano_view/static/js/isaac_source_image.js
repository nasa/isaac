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

/* View source image with OpenSeaDragon */

function parseUrlParameters() {
    var url;
    if (window.location.hash.length > 0) {
        // Prefered method since parameters aren't sent to server
        url = window.location.hash.slice(1);
    } else {
        url = window.location.search.slice(1);
    }
    if (!url) {
        // Display error if no configuration parameters are specified
        anError('No configuration options were specified.');
        return;
    }
    url = url.split('&');
    var configFromUrl = {};
    for (var i = 0; i < url.length; i++) {
        var option = url[i].split('=')[0];
        var value = url[i].split('=')[1];
        if (value == '')
            continue; // Skip options with empty values in url config
        switch(option) {
            case 'scene':
            case 'imageId':
                configFromUrl[option] = decodeURIComponent(value);
                break;
            default:
                anError('An invalid configuration parameter was specified: ' + option);
                return;
        }
    }

    return configFromUrl;
}

function initIsaacSourceImage() {
    var configFromUrl = parseUrlParameters();

    var viewer = OpenSeadragon({
        id: 'container',
        prefixUrl: '../../media/openseadragon/',
        tileSources: '../../source_images/' + configFromUrl['scene'] + '/'
	    + configFromUrl['imageId'] + '.dzi'
    });
    var anno = OpenSeadragon.Annotorious(viewer);

    Annotorious.SelectorPack(anno, {
	tools: ['point']
    });

    console.log(document.getElementById('isaac-toolbar-container'));
    Annotorious.Toolbar(anno, document.getElementById('isaac-toolbar-container'));
}

initIsaacSourceImage();
