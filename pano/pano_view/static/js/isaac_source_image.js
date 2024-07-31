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

const ISAAC_STORAGE_ROOT_KEY = 'isaac_anno';
const ISAAC_DEFAULT_CONTENT_KEY = 'isaac_anno_default';

/* The index used to remember what annotation we're reviewing using
 * the Next/Previous buttons. */
var ISAAC_REVIEW_INDEX = 0;

/* Simple should-be-unique ID for this window. Used to distinguish
 * between local storage updates coming from this window vs. from
 * other windows. The timestamp has millisecond precision. */
const ISAAC_WINDOW_ID = Date.now();

/* Return a configuration dict parsed from the hash component of the
 * window's location URL. Falls back to the query parameter component
 * if the hash component is not set. The parsing is strict and will
 * only accept fields 'scene' and 'imageId'. Example: URL
 * 'https://some.url/#scene=x&imageId=y' -> {scene: 'x', imageId:
 * 'y'}. */
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
            case 'slug':
                configFromUrl[option] = decodeURIComponent(value);
                break;
            default:
                anError('An invalid configuration parameter was specified: ' + option);
                return;
        }
    }

    return configFromUrl;
}

/* Return obj[field] if set. Otherwise, set obj[field] = defaultValue
 * and return defaultValue. */
function isaacSetDefault(obj, field, defaultValue) {
    if (obj[field] == undefined) {
        obj[field] = defaultValue;
        return defaultValue;
    }
    return obj[field];
}

/* Return the application's storage object (parsed from a JSON string stored
 * at the ISAAC_STORAGE_ROOT_KEY in HTML5 window.localStorage). */
function isaacStorageGetRoot() {
    var storageItemText = window.localStorage.getItem(ISAAC_STORAGE_ROOT_KEY);
    if (storageItemText == null) {
        storageItemText = window.localStorage.getItem(ISAAC_DEFAULT_CONTENT_KEY);
    }
    if (storageItemText == null) {
        storageItemText = "{}";
    }
    return JSON.parse(storageItemText);
}

/* Set the applications storage object to `obj`. */
function isaacStorageSetRoot(obj) {
    obj.source_window_id = ISAAC_WINDOW_ID;
    window.localStorage.setItem(ISAAC_STORAGE_ROOT_KEY, JSON.stringify(obj));
}

/* Return the result of drilling down into `obj`, treating `fieldPath`
 *  as an array of references to retrieve. Example:
 *  isaacGetFieldPath(obj, ["foo", 1]) -> obj.foo[1]. */
function isaacGetFieldPath(obj, fieldPath) {
    for (fieldElt of fieldPath) {
        obj = obj[fieldElt];
        if (obj == undefined) {
            return undefined;
        }
    }
    return obj;
}

/* Return the storage object value at `fieldPath`, where `fieldPath`
 * is an array of field names used to drill down into the storage
 * object. */
function isaacStorageGet(fieldPath) {
    var storageItem = isaacStorageGetRoot();
    return isaacGetFieldPath(storageItem, fieldPath);
}

/* Companion to `isaacStorageGet()`. Set the storage object value at
 * `fieldPath` to `value`. */
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

/* Return the storage object value at `fieldPath`. If not set, set it
 * to defaultValue and return defaultValue.
 */
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

/* Delete the storage object value at `fieldPath`. */
function isaacStorageDelete(fieldPath) {
    var storageItem = isaacStorageGetRoot();

    var obj = storageItem;
    for (fieldElt of fieldPath.slice(0, -1)) {
        obj = isaacSetDefault(obj, fieldElt, {});
    }
    delete obj[fieldPath[fieldPath.length - 1]];

    isaacStorageSetRoot(storageItem);
}

/* Convert `data` to a JSON string and trigger the browser to download
 * it as an attachment named `fileName`. Note: This has a side-effect
 * of deleting the `source_window_id` field of `data`, which we don't want to save. */
function isaacSaveData(data, fileName) {
    delete data.source_window_id;
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

/* The maximum depth of the history undo and redo stacks. */
var ISAAC_HISTORY_MAX_LENGTH = 10;

/* Set the disabled status of the Undo and Redo buttons based on the
 * annotation history stacks. */
function isaacHistoryUpdateEnabled(history) {
    document.getElementById('isaac-undo').disabled = (history.undoStack.length == 0);
    document.getElementById('isaac-redo').disabled = (history.redoStack.length == 0);
}

/* Log a debug message about the annotation history stacks. */
function isaacHistoryDebug(history) {
    console.log("#undo=" + history.undoStack.length + " #redo=" + history.redoStack.length)
}

/* Save a new annotation state to the history and update the disabled
 * state of the Undo/Redo buttons. */
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

/* Update the Annotorious part of the UI to reflect a new annotation
 * state. This doesn't modify the storage object or update the
 * history.
 *
 * @param history A history object with fields 'current', 'undoStack', 'redoStack'.
 * @param state The annotation state (corresponds to the complete storage object).
 * @param imageStoragePath A path within `state` referencing the annotation state for the current image.
 * @param anno A reference to the live instance of the Annotorious viewer.
 */
function isaacRenderState(history, state, imageStoragePath, anno) {
    var imageAnnotations = isaacGetFieldPath(state, imageStoragePath) || [];
    anno.setAnnotations(imageAnnotations);
    isaacHistoryUpdateEnabled(history);
}

/* Perform an undo operation. This updates the current annotation state, adjusts the history undo/redo stacks,
 * saves the annotation state to the storage object, and updates the UI.
 *
 * @param history A history object with fields 'current', 'undoStack', 'redoStack'.
 * @param imageStoragePath A path within the storage object referencing the annotation state for the current image.
 * @param anno A reference to the live instance of the Annotorious viewer.
 */
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

/* Perform a redo operation. Params and effects similar to undo (see above). */
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

/* Record an Annotorious edit operation. This updates the storage object and the
   history undo/redo stacks, and updates the non-Annotorious part of the UI.
   (The Annotorious part of the UI is assumed to be the source of the edit and
   shouldn't need to be updated.) */
function isaacUpdateAnnotations(fieldPath, value, history) {
    isaacStorageSet(fieldPath, value);
    isaacHistorySaveState(history, isaacStorageGetRoot());
}

/* Return a Promise that resolves to the text content of `file` (a
   local file picked by the user for upload using the "Load"
   button). */
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

/* Change the annotation review index by `delta`. This changes which
 * annotation is highlighted and pans to the highlighted annotation.
 *
 * @param imageStoragePath A path within the storage object referencing the annotation state for the current image.
 * @param anno A reference to the live instance of the Annotorious viewer.
 * @param delta Specify delta = 1 for the Next button or -1 for the Previous button.
 */
function isaacReviewUpdater(imageStoragePath, anno, delta) {
    var annotations = isaacStorageGet(imageStoragePath);
    if (!annotations) return;
    ISAAC_REVIEW_INDEX = (ISAAC_REVIEW_INDEX + delta + annotations.length) % annotations.length;
    var annoId = annotations[ISAAC_REVIEW_INDEX].id;
    anno.selectAnnotation(annoId);
    anno.panTo(annoId);
}

/* Process an annotation update that comes from the storage
 * object. This updates the history and both the Annotorious and
 * non-Annotorious parts of the UI.
 *
 * @param history A history object with fields 'current', 'undoStack', 'redoStack'.
 * @param imageStoragePath A path within the storage object referencing the annotation state for the current image.
 * @param anno The live instance of the Annotorious viewer.
 */
function isaacProcessStorageUpdate(history, imageStoragePath, anno) {
    var storageItem = isaacStorageGetRoot();
    isaacRenderState(history, storageItem, imageStoragePath, anno);
    isaacHistorySaveState(history, storageItem);
}

/* Handle an annotation update that comes from the storage
 * object. This updates the history and both the Annotorious and
 * non-Annotorious parts of the UI.
 *
 * @param history A history object with fields 'current', 'undoStack', 'redoStack'.
 * @param imageStoragePath A path within the storage object referencing the annotation state for the current image.
 * @param anno The live instance of the Annotorious viewer.
 */
function isaacHandleStorageEvent(history, imageStoragePath, anno, event) {
    if (event.key !== null && event.key != ISAAC_STORAGE_ROOT_KEY) {
        // Ignore storage events unless they affect our key. The event key will be null
        // for a clear event that affects all keys.
        return;
    }
    var storageItem = JSON.parse(event.newValue);
    if (storageItem.source_window_id == ISAAC_WINDOW_ID) {
        // Ignore storage events that were triggered by changes made in this
        // window -- they will already have been explicitly handled, or not,
        // as needed.
        return;
    }
    isaacProcessStorageUpdate(history, imageStoragePath, anno);
}

/* Configure handlers for the Load, Save, and Clear buttons.
 *
 * @param storageUpdateHandler A callback invoked with no arguments when the storage object changes.
 */
function isaacConfigureLoadSaveClear(storageUpdateHandler) {
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
            storageUpdateHandler();
        }
    });

    document.getElementById('isaac-save').addEventListener('click', function(event) {
        isaacSaveData(isaacStorageGetRoot(), 'isaac_iss_annotations.json');
    });
    document.getElementById('isaac-clear').addEventListener('click', function(event) {
        isaacStorageSetRoot({});
        storageUpdateHandler();
    });
}

/* Perform overall initialization for the ISAAC pano tour source image
   viewer. */
function initIsaacSourceImage() {
    var configFromUrl = parseUrlParameters();

    // Initialize OpenSeaDragon viewer and Annotorious plugin
    var viewer = OpenSeadragon({
        id: 'container',
        prefixUrl: '../media/openseadragon/',
        tileSources: '../source_images/' + configFromUrl['scene'] + '/'
            + configFromUrl['imageId'] + '.dzi',
        maxZoomPixelRatio: 5
    });
    const annoConfig = {
        allowEmpty: true
    };
    var anno = OpenSeadragon.Annotorious(viewer, annoConfig);
    // anno.setDrawingEnabled(true);
    var history = {
        'current': null,
        'undoStack': [],
        'redoStack': []
    };
    // anno.removeDrawingTool('rect');
    // anno.removeDrawingTool('polygon');

    // Export symbols for debugging
    window.configFromUrl = configFromUrl;
    window.viewer = viewer;
    window.anno = anno;
    window.isaacHistory = history;

    // Configure extra point drawing tool (default tools are rect and polygon only)
    Annotorious.SelectorPack(anno, {
        tools: ['point']
    });

    // Set slug text
    slugText = configFromUrl['slug'].replaceAll("_", " ");
    document.getElementById('isaac-slug').textContent = slugText;
    document.title = slugText + ": ISAAC ISS Tour";

    // Configure other button handlers
    document.getElementById('isaac-raw-anchor').href = '../../source_images/'
        + configFromUrl['scene'] + '/'
        + configFromUrl['imageId'] + '.jpg';
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
            anno.setDrawingTool('rect');
            anno.setDrawingEnabled(true);
        }
    );

    var storageUpdateHandler = () => isaacProcessStorageUpdate(history, imageStoragePath, anno)
    isaacConfigureLoadSaveClear(storageUpdateHandler);

    var reviewUpdater = delta => (event => isaacReviewUpdater(imageStoragePath, anno, delta));
    document.getElementById('isaac-previous').addEventListener('click', reviewUpdater(-1));
    document.getElementById('isaac-next').addEventListener('click', reviewUpdater(1))

    // Restore annotations on this image from HTML5 local storage
    isaacProcessStorageUpdate(history, imageStoragePath, anno);

    // Handle storage changes triggered by other windows
    window.addEventListener(
        "storage",
        (event) => isaacHandleStorageEvent(history, imageStoragePath, anno, event)
    );

    // Save subsequent drawing events to HTML5 local storage
    var annoEventHandler = annotation => isaacUpdateAnnotations(imageStoragePath, anno.getAnnotations(), history);
    anno.on('createAnnotation', annoEventHandler);
    anno.on('updateAnnotation', annoEventHandler);
    anno.on('deleteAnnotation', annoEventHandler);
}
