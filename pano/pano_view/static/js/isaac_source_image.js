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

var ISAAC_STORAGE_ROOT_KEY = 'annotations';

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
    var storageItemText = window.localStorage.getItem(ISAAC_STORAGE_ROOT_KEY) || "{}";
    return JSON.parse(storageItemText);
}

function isaacStorageSetRoot(obj) {
    window.localStorage.setItem(ISAAC_STORAGE_ROOT_KEY, JSON.stringify(obj));
}

function isaacGetFieldPath(obj, fieldPath) {
    for (fieldElt of fieldPath) {
        obj = obj[fieldElt];
        if (obj == undefined) {
            return undefined;
        }
    }
    return obj;
}

function isaacStorageGet(fieldPath) {
    var storageItem = isaacStorageGetRoot();
    return isaacGetFieldPath(storageItem, fieldPath);
}

function isaacStorageSet(fieldPath, value) {
    if (fieldPath.length == 0) {
        isaacStorageSetRoot(value);
        return;
    }

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

var ISAAC_HISTORY_MAX_LENGTH = 10;

function isaacHistoryUpdateEnabled(history) {
    document.getElementById('isaac-undo').disabled = (history.undoStack.length == 0);
    document.getElementById('isaac-redo').disabled = (history.redoStack.length == 0);
}

function isaacHistoryDebug(history) {
    console.log("#undo=" + history.undoStack.length + " #redo=" + history.redoStack.length)
}

function isaacHistorySaveState(history, state) {
    if (history.current != null) {
        history.undoStack.push(history.current);
    }
    if (history.undoStack.length > ISAAC_HISTORY_MAX_LENGTH) {
        history.undoStack.shift();
    }
    history.current = state;
    history.redoStack = [];

    isaacHistoryUpdateEnabled(history);
    isaacHistoryDebug(history);
}

function isaacRenderState(history, state, imageStoragePath, anno) {
    var imageAnnotations = isaacGetFieldPath(state, imageStoragePath) || [];
    anno.setAnnotations(imageAnnotations);
    isaacHistoryUpdateEnabled(history);
}

function isaacHistoryUndo(history, imageStoragePath, anno) {
    if (history.undoStack.length == 0) {
        console.log('got undo request with no undo stack, should never happen');
        return;
    }
    history.redoStack.push(history.current);
    history.current = history.undoStack.pop();

    isaacStorageSetRoot(history.current);

    isaacRenderState(history, history.current, imageStoragePath, anno);
    isaacHistoryDebug(history);
}

function isaacHistoryRedo(history, imageStoragePath, anno) {
    if (history.redoStack.length == 0) {
        console.log('got redo request with no redo stack, should never happen');
        return;
    }
    history.undoStack.push(history.current);
    history.current = history.redoStack.pop();

    isaacStorageSetRoot(history.current);

    isaacRenderState(history, history.current, imageStoragePath, anno);
    isaacHistoryDebug(history);
}

function isaacUpdateAnnotations(fieldPath, value, history) {
    isaacStorageSet(fieldPath, value);
    isaacHistorySaveState(history, isaacStorageGetRoot());
}

function isaacReadAsText(file) {
    // Always return a Promise
    return new Promise((resolve, reject) => {
        let content = '';
        const reader = new FileReader();
        // Wait till complete
        reader.onloadend = function(e) {
            resolve(e.target.result);
        };
        // Make sure to handle error states
        reader.onerror = function(e) {
            reject(e);
        };
        reader.readAsText(file);
    });
}

function initIsaacSourceImage() {
    var configFromUrl = parseUrlParameters();

    // Initialize OpenSeaDragon viewer and Annotorious plugin
    var viewer = OpenSeadragon({
        id: 'container',
        prefixUrl: '../../media/openseadragon/',
        tileSources: '../../source_images/' + configFromUrl['scene'] + '/'
            + configFromUrl['imageId'] + '.dzi',
        maxZoomPixelRatio: 5
    });
    var anno = OpenSeadragon.Annotorious(viewer);
    var history = {
        'current': null,
        'undoStack': [],
        'redoStack': []
    };
    anno.removeDrawingTool('rect');
    anno.removeDrawingTool('polygon');

    // Export symbols for debugging
    window.configFromUrl = configFromUrl;
    window.viewer = viewer;
    window.anno = anno;
    window.isaacHistory = history;

    // Configure extra point drawing tool (default tools are rect and polygon only)
    Annotorious.SelectorPack(anno, {
        tools: ['point']
    });

    // Configure other button handlers
    var imageStoragePath = [configFromUrl.scene, configFromUrl.imageId];
    document.getElementById('isaac-undo').addEventListener(
        'click',
        event => isaacHistoryUndo(history, imageStoragePath, anno)
    );
    document.getElementById('isaac-redo').addEventListener(
        'click',
        event => isaacHistoryRedo(history, imageStoragePath, anno)
    );

    document.getElementById('isaac-add').addEventListener(
        'click',
        event => {
            anno.setDrawingTool('point');
            anno.setDrawingEnabled(true);
        }
    );

    var isaacLoadInput = document.getElementById('isaac-load-input');
    document.getElementById('isaac-load').addEventListener('click', function(event) {
        isaacLoadInput.click();
    });
    isaacLoadInput.addEventListener('change', async function(event) {
        console.log('load change event');
        console.log(isaacLoadInput);
        if (isaacLoadInput.files.length > 0) {
            var loadFile = isaacLoadInput.files[0];
            var loadText = await isaacReadAsText(loadFile);
            var storageItem = JSON.parse(loadText);
            isaacStorageSetRoot(storageItem);
            isaacHistorySaveState(history, storageItem);
            isaacRenderState(history, storageItem, imageStoragePath, anno);
        }
    });

    document.getElementById('isaac-save').addEventListener('click', function(event) {
        isaacSaveData(isaacStorageGetRoot(), 'annotations.json');
    });
    document.getElementById('isaac-clear').addEventListener('click', function(event) {
        isaacRenderState(history, {}, imageStoragePath, anno);
        isaacUpdateAnnotations([], {}, history);
    });

    var reviewIndex = 0;
    var reviewUpdater = function(delta) {
        return function(event) {
            var annotations = isaacStorageGet(imageStoragePath);
            if (!annotations) return;
            reviewIndex = (reviewIndex + delta + annotations.length) % annotations.length;
            var annoId = annotations[reviewIndex].id;
            anno.selectAnnotation(annoId);
            anno.panTo(annoId);
        }
    }
    document.getElementById('isaac-previous').addEventListener('click', reviewUpdater(-1));
    document.getElementById('isaac-next').addEventListener('click', reviewUpdater(1))

    // Restore annotations on this image from HTML5 local storage
    var storageItem = isaacStorageGetRoot();
    isaacRenderState(history, storageItem, imageStoragePath, anno);
    isaacHistorySaveState(history, storageItem);

    // Save subsequent drawing events to HTML5 local storage
    anno.on('createAnnotation', function(annotation) {
        isaacUpdateAnnotations(imageStoragePath, anno.getAnnotations(), history);
    });
    anno.on('updateAnnotation', function(annotation) {
        isaacUpdateAnnotations(imageStoragePath, anno.getAnnotations(), history);
    });
    anno.on('deleteAnnotation', function(annotation) {
        isaacUpdateAnnotations(imageStoragePath, anno.getAnnotations(), history);
    });
}

initIsaacSourceImage();
