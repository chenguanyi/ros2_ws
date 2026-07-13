import cv2, numpy as np, sys, os, time

# Load image
img = cv2.imread("/home/orangepi/models/test.png")
if img is None:
    img = cv2.imread("/home/orangepi/models/动物体态 .png")
if img is None:
    print("FAIL: cannot load image"); sys.exit(1)
print(f"Image size: {img.shape[1]}x{img.shape[0]}")

# Init RKNN
from rknnlite.api import RKNNLite
rknn = RKNNLite(verbose=False)
rknn.load_rknn("/home/orangepi/models/h2025_yolov5s_i8.rknn")
rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)

# Test 1: single inference
from rknn_yolo_detector_pkg.yolov5_postprocess import YoloV5PostProcessor, letterbox, parse_anchor_string

input_w, input_h = 640, 640
post = YoloV5PostProcessor(input_w, input_h, 5, 0.25, 0.45,
    parse_anchor_string("10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326"))

preprocessed, info = letterbox(img, input_w, input_h)
inp = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2RGB)
inp = np.expand_dims(np.ascontiguousarray(inp, dtype=np.uint8), axis=0)

t0 = time.perf_counter()
outputs = rknn.inference(inputs=[inp])
t1 = time.perf_counter()
dets = post([np.asarray(o) for o in outputs], info)
t2 = time.perf_counter()
print(f"Single frame: preprocess=0ms inference={(t1-t0)*1000:.1f}ms postprocess={(t2-t1)*1000:.1f}ms detections={len(dets)}")
for d in dets:
    print(f"  class={d.class_id} conf={d.confidence:.3f} bbox=({d.x1:.0f},{d.y1:.0f},{d.x2:.0f},{d.y2:.0f})")

# Draw detections
for d in dets:
    cv2.rectangle(img, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)), (0,255,0), 3)
    cv2.putText(img, f"{d.class_id}:{d.confidence:.2f}", (int(d.x1), max(0,int(d.y1)-5)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
cv2.imwrite("/home/orangepi/models/test_output.jpg", img)
print("Saved: /home/orangepi/models/test_output.jpg")

# Test 2: simulated movement (pan image horizontally)
print("\n--- Simulated horizontal movement ---")
img_h, img_w = img.shape[:2]
fps_list = []
for offset in range(0, img_w - 640 + 1, 20):
    crop = img[:, offset:offset+640]
    if crop.shape[1] < 640: break
    pre, inf = letterbox(crop, input_w, input_h)
    inp2 = cv2.cvtColor(pre, cv2.COLOR_BGR2RGB)
    inp2 = np.expand_dims(np.ascontiguousarray(inp2, dtype=np.uint8), axis=0)
    t0 = time.perf_counter()
    outs = rknn.inference(inputs=[inp2])
    t1 = time.perf_counter()
    fps = 1.0 / (t1 - t0) if (t1 - t0) > 0 else 0
    fps_list.append(fps)
    dets2 = post([np.asarray(o) for o in outs], inf)
    print(f"  offset={offset:3d}px inference={(t1-t0)*1000:.1f}ms fps={fps:.1f} detections={len(dets2)}")

if fps_list:
    print(f"\nInference FPS: mean={np.mean(fps_list):.1f} min={np.min(fps_list):.1f} max={np.max(fps_list):.1f}")

# Test 3: blur simulation
print("\n--- Blur simulation ---")
for blur in [0, 15, 31, 51]:
    if blur > 0:
        blurred = cv2.GaussianBlur(img, (blur, blur), 0)
    else:
        blurred = img.copy()
    pre, inf = letterbox(blurred, input_w, input_h)
    inp3 = cv2.cvtColor(pre, cv2.COLOR_BGR2RGB)
    inp3 = np.expand_dims(np.ascontiguousarray(inp3, dtype=np.uint8), axis=0)
    outs = rknn.inference(inputs=[inp3])
    dets3 = post([np.asarray(o) for o in outs], inf)
    print(f"  blur_kernel={blur:2d} detections={len(dets3)}: {[(d.class_id, round(d.confidence,3)) for d in dets3]}")

rknn.release()
