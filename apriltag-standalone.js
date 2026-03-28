/**
 * Standalone AprilTag detector wrapper (no Web Workers)
 * Adapted from arenaxr/apriltag-js-standalone
 */
class Apriltag {
    constructor(onDetectorReadyCallback) {
        this.onDetectorReadyCallback = onDetectorReadyCallback;

        // detector options
        this._opt = {
          quad_decimate: 2.0,
          quad_sigma: 0.0,
          nthreads: 1,
          refine_edges: 1,
          max_detections: 0,
          return_pose: 0,  // Don't compute pose by default
          return_solutions: 0
        }

        let _this = this;
        AprilTagWasm().then(function (Module) {
            console.log("Apriltag WASM module loaded.");
            _this.onWasmInit(Module);
        });
    }

    onWasmInit(Module) {
        this._Module = Module;
        this._init = Module.cwrap('atagjs_init', 'number', []);
        this._destroy = Module.cwrap('atagjs_destroy', 'number', []);
        this._set_detector_options = Module.cwrap('atagjs_set_detector_options', 'number', 
            ['number', 'number', 'number', 'number', 'number', 'number', 'number']);
        this._set_pose_info = Module.cwrap('atagjs_set_pose_info', 'number', 
            ['number', 'number', 'number', 'number']);
        this._set_img_buffer = Module.cwrap('atagjs_set_img_buffer', 'number', 
            ['number', 'number', 'number']);
        this._atagjs_set_tag_size = Module.cwrap('atagjs_set_tag_size', null, 
            ['number', 'number']);
        this._detect = Module.cwrap('atagjs_detect', 'number', []);

        // Initialize detector
        this._init();

        // Set detector options
        this._set_detector_options(
          this._opt.quad_decimate,
          this._opt.quad_sigma,
          this._opt.nthreads,
          this._opt.refine_edges,
          this._opt.max_detections,
          this._opt.return_pose,
          this._opt.return_solutions);

        this.onDetectorReadyCallback();
    }

    detect(grayscaleImg, imgWidth, imgHeight) {
        // Allocate buffer for image
        let imgBuffer = this._set_img_buffer(imgWidth, imgHeight, imgWidth);
        if (imgWidth * imgHeight < grayscaleImg.length) {
            console.error("Image data too large.");
            return [];
        }
        
        // Copy grayscale image data to WASM memory
        this._Module.HEAPU8.set(grayscaleImg, imgBuffer);
        
        // Run detection
        let strJsonPtr = this._detect();
        
        // Read string length from returned struct
        let strJsonLen = this._Module.getValue(strJsonPtr, "i32");
        if (strJsonLen == 0) {
            return [];
        }
        
        // Read string pointer from struct
        let strJsonStrPtr = this._Module.getValue(strJsonPtr + 4, "i32");
        const strJsonView = new Uint8Array(this._Module.HEAP8.buffer, strJsonStrPtr, strJsonLen);
        
        // Build JSON string
        let detectionsJson = '';
        for (let i = 0; i < strJsonLen; i++) {
            detectionsJson += String.fromCharCode(strJsonView[i]);
        }
        
        // Parse and return detections
        try {
            let detections = JSON.parse(detectionsJson);
            return detections;
        } catch(e) {
            console.error("Failed to parse detections:", e);
            return [];
        }
    }

    set_camera_info(fx, fy, cx, cy) {
        this._set_pose_info(fx, fy, cx, cy);
    }

    set_tag_size(tagid, size) {
        this._atagjs_set_tag_size(tagid, size);
    }

    set_max_detections(maxDetections) {
        this._opt.max_detections = maxDetections;
        this._set_detector_options(
          this._opt.quad_decimate,
          this._opt.quad_sigma,
          this._opt.nthreads,
          this._opt.refine_edges,
          this._opt.max_detections,
          this._opt.return_pose,
          this._opt.return_solutions);
    }

    set_return_pose(returnPose) {
        this._opt.return_pose = returnPose;
        this._set_detector_options(
          this._opt.quad_decimate,
          this._opt.quad_sigma,
          this._opt.nthreads,
          this._opt.refine_edges,
          this._opt.max_detections,
          this._opt.return_pose,
          this._opt.return_solutions);
    }
}
