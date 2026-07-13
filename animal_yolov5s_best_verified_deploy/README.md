# Orange Pi RK3588 deployment package

Use `animal_yolov5s_best_fp_rk3588.rknn` as the default model. It was converted from this task's best YOLOv5 model and validated in RKNN Toolkit2 2.3.2's build-time simulator.

## Verification result

- Output shape: `(1, 25200, 10)`
- Maximum objectness: `0.755371`
- Maximum class probability: `0.996582`
- Candidates at confidence `0.25`: `14`
- SHA-256: `05003608d6cabd67da4f1140fd79ad96f74069fa46a93f0759724b1e75ccb23b`

## Deployment contract

- Input: BGR `uint8` image, resized to `640 x 640`.
- Input normalization is built into the RKNN model (`value / 255`). Do not divide by 255 again in the Orange Pi application.
- Output: one `float32` tensor in YOLOv5 decoded format `(1, 25200, 10)`.
- Class order is defined in `classes.txt`.

## Important

The separately generated INT8 candidate is deliberately excluded from this package. Its WSL simulator test reproduced the known fault: coordinates were nonzero but objectness and every class channel were zero, leaving zero candidates even at threshold `0.001`.

Before deployment, verify the model hash on the Orange Pi and ensure its RKNN Runtime/Lite2 version is compatible with RKNN Toolkit2 2.3.2.
