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

function isaacSetDefault(obj, field, defaultValue) {
    if (obj[field] == undefined) {
	obj[field] = defaultValue;
	return defaultValue;
    }
    return obj[field];
}

function isaacStorageGetRoot() {
    var storageItemText = window.localStorage.getItem('annotations') || "{}";
    return JSON.parse(storageItemText);
}

function isaacStorageSetRoot(obj) {
    window.localStorage.setItem('annotations', JSON.stringify(obj));
}

function isaacStorageGet(fieldPath) {
    var storageItem = isaacStorageGetRoot();

    var obj = storageItem;
    for (fieldElt of fieldPath) {
	obj = obj[fieldElt];
	if (obj == undefined) {
	    return undefined;
	}
    }
    return obj;
}

function isaacStorageSet(fieldPath, value) {
    var storageItem = isaacStorageGetRoot();

    var obj = storageItem;
    for (fieldElt of fieldPath.slice(0, -1)) {
	obj = isaacSetDefault(obj, fieldElt, {});
    }
    obj[fieldPath[fieldPath.length - 1]] = value;

    isaacStorageSetRoot(storageItem);
}

function isaacStorageSetDefault(fieldPath, defaultValue) {
    var storageItem = isaacStorageGetRoot();

    var obj = storageItem;
    for (fieldElt of fieldPath.slice(0, -1)) {
	obj = isaacSetDefault(obj, fieldElt, {});
    }

    var fieldLast = fieldPath[fieldPath.length - 1];
    if (obj[fieldLast] == undefined) {
	obj[fieldLast] = defaultValue;
	isaacStorageSetRoot(storageItem);
	return defaultValue;
    }
    return obj[fieldLast];
}

function isaacStorageDelete(fieldPath) {
    var storageItem = isaacStorageGetRoot();

    var obj = storageItem;
    for (fieldElt of fieldPath.slice(0, -1)) {
	obj = isaacSetDefault(obj, fieldElt, {});
    }
    delete obj[fieldPath[fieldPath.length - 1]];

    isaacStorageSetRoot(storageItem);
}

function isaacSaveData(data, fileName) {
    var json = JSON.stringify(data, null, 4);
    var blob = new Blob([json], {type: 'application/json'});
    var url = window.URL.createObjectURL(blob);

    var a = document.createElement("a");
    a.href = url;
    a.download = fileName;
    a.style = "display: none";
    document.body.appendChild(a);

    a.click();

    window.URL.revokeObjectURL(url);
    a.remove();
}

function initIsaacSourceImage() {
    var configFromUrl = parseUrlParameters();

    // Initialize OpenSeaDragon viewer and Annotorious plugin
    var viewer = OpenSeadragon({
        id: 'container',
        prefixUrl: '../../media/openseadragon/',
        tileSources: '../../source_images/' + configFromUrl['scene'] + '/'
	    + configFromUrl['imageId'] + '.dzi'
    });
    var anno = OpenSeadragon.Annotorious(viewer);

    // Export symbols for debugging
    window.configFromUrl = configFromUrl;
    window.viewer = viewer;
    window.anno = anno;

    // Configure extra point drawing tool (default tools are rect and polygon only)
    Annotorious.SelectorPack(anno, {
	tools: ['point']
    });

    // Create toolbar
    Annotorious.Toolbar(anno, document.getElementById('isaac-toolbar-container'));

    // Configure other button handlers
    document.getElementById('isaac-save').addEventListener("click", function(event) {
	isaacSaveData(isaacStorageGetRoot(), 'annotations.json');
    });
    document.getElementById('isaac-clear').addEventListener("click", function(event) {
	isaacStorageSetRoot({});
	anno.clearAnnotations();
    });

    // Restore annotations on this image from HTML5 local storage
    var imageAnnotations = isaacStorageGet([configFromUrl.scene, configFromUrl.imageId]);
    if (imageAnnotations) {
	anno.setAnnotations(imageAnnotations);
    }

    // Save subsequent drawing events to HTML5 local storage
    var imageStoragePath = [configFromUrl.scene, configFromUrl.imageId];
    anno.on('createAnnotation', function(annotation) {
	// isaacStorageSet([configFromUrl.scene, configFromUrl.imageId, annotation.id], annotation);
	var annotations = isaacStorageSetDefault(imageStoragePath, []);
	annotations.push(annotation);
	isaacStorageSet(imageStoragePath, annotations);
    });
    anno.on('updateAnnotation', function(annotation) {
	var annotations = isaacStorageSetDefault(imageStoragePath, []);

	var idx = annotations.findIndex(a => (a.id == annotation.id));
	if (idx == undef) {
	    console.log('warning: updated element not found, creating instead');
	    annotations.push(annotation);
	} else {
	    annotations[idx] = annotation;
	}

	isaacStorageSet(imageStoragePath, annotations);
    });
    anno.on('deleteAnnotation', function(annotation) {
	var annotations = isaacStorageSetDefault(imageStoragePath, []);

	annotations = annotations.filter(a => (a.id != annotation.id));

	isaacStorageSet(imageStoragePath, annotations);
    });
}

initIsaacSourceImage();
