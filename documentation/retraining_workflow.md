# Ball Detector Training and Retraining Workflow

## Overview

Train or Retrain the YOLOv8n ball/cup detector with new images captured from the
current camera position, then export at 640x640 input size for improved
detection accuracy.

**Tools needed:**
- Orin Nano host (not Docker) — image capture and training
- Roboflow account — annotation and augmentation
- tools.luxonis.com — blob conversion for MyriadX
- Docker container — deployment

---

## Phase 1 — Capture Training Images

Run on Orin Nano **host** (not in Docker). Requires VNC display.

```bash
python3 ~/temp/capture_oak_v2.py
```

Controls: SPACE=save frame, Q=quit

**What to capture (aim for 200-300 new raw images):**
- Ball at centre of cup
- Ball at left, right, near, far edges
- Ball at all four diagonal corners
- Ball partially off the cup edge
- No ball on cup (negative examples — 20-30 images)
- Arm in multiple poses from balance_v1.yaml (not just pose 1)
- With and without white background sheet
- Different lighting if possible

**Where images are saved:**
```
~/training_data/images/
```

---

## Phase 2 — Upload and Annotate in Roboflow

1. Go to roboflow.com and log into the existing project
2. Click **Upload** and select all images from `~/training_data/images/`
3. For each image, draw bounding boxes:
   - **ball** — tight box around the ball bearing
   - **cup** — tight box around the cup plate
   - If no ball visible, draw only the cup box
   - If neither visible, add image with no annotations (negative example)

**Tips for faster annotation:**
- 
- Tab key moves to next image
- Click and drag to draw box, then select class. Do not use any of the other features in roboflow that create non-square bounding boxes (for this initial model)

4. Once annotated, click **Generate Dataset**:
   - Train/Valid/Test split: 80/15/5 (default)
   - Augmentations: enable brightness, blur, rotation, noise
   - This multiplies the images automatically

5. Click **Export Dataset**:
   - Format: YOLOv8
   - Download as zip to Orin


---

## Phase 3 — Prepare Dataset on Orin

```bash
cd ~/temp
mkdir -p dataset_640
unzip ball-detection-*.zip -d dataset_640/
ls dataset_640/
# Should show: data.yaml  train/  valid/  test/
```

Edit `data.yaml` to confirm paths are correct:
```bash
cat dataset_640/data.yaml
```

Should show something like:
```yaml
train: train/images
val: valid/images
test: test/images
nc: 2
names: ['ball', 'cup']
```

---

## Phase 4 — Train on Orin Nano

Run on Orin Nano **host** (not Docker). Uses Jetson GPU via torch 2.8.

```bash
cd ~/temp

yolo train \
  model=yolov8n.pt \
  data=dataset_640/data.yaml \
  epochs=100 \
  imgsz=640 \
  batch=4 \
  device=0 \
  project=runs/detect/ball_training_640 \
  name=run1 \
  patience=20
```

**Parameters explained:**
- `imgsz=640` — larger than previous 416, better detection of small ball
- `batch=4` — reduced from 8 to fit 640x640 in Orin GPU memory
- `patience=20` — early stop if no improvement for 20 epochs
- `device=0` — use Orin GPU (CUDA)

**Monitor training:**
```bash
# In another terminal
watch -n 10 tail -5 ~/temp/runs/detect/ball_training_640/run1/results.csv
```

Target: `mAP50 > 0.95` (previous run achieved 0.994)

**Expected time:** 2-3 hours at 640x640 (vs 1.5 hours at 416x416)

**Results location:**
```
~/temp/runs/detect/ball_training_640/run1/weights/best.pt
```

---

## Phase 5 — Validate Training Results

```bash
cd ~/temp

yolo val \
  model=runs/detect/ball_training_640/run1/weights/best.pt \
  data=dataset_640/data.yaml \
  device=0
```

Check mAP50 — should be above 0.95. If not, consider:
- More training epochs (increase epochs, rerun)
- More training data in problem areas
- Check for annotation errors


---

## Phase 6 — Convert to ONNX

```bash
cd ~/temp

yolo export \
  model=runs/detect/ball_training_640/run1/weights/best.pt \
  format=onnx \
  imgsz=640 \
  opset=12

# Output:
# runs/detect/ball_training_640/run1/weights/best.onnx
```

---

## Phase 7 — Convert ONNX to MyriadX Blob

1. Go to tools.luxonis.com in browser on your Mac
2. Select **YOLOv8 (detection only)** from dropdown
3. Select **RVC2** (for OAK-D Lite)
4. Upload `best.pt` from:
   ```
   ~/temp/runs/detect/ball_training_640/run1/weights/best.pt
   ```
   (scp to Mac first if needed)
5. Input image shape: `640 640` (space-separated, no commas)
6. Click Convert and wait
7. Download `result.zip`

Extract on Orin host:
```bash
cd ~/temp
mkdir luxonis_export_640
unzip ~/Downloads/result.zip -d luxonis_export_640/
ls luxonis_export_640/
# Should show: best.json  best_openvino_2022.1_6shave.blob  best.onnx  best.xml  best.bin
```

---

## Phase 8 — Deploy to Container

Copy blob and JSON to the balance package models directory:
```bash
cp ~/temp/luxonis_export_640/best_openvino_2022.1_6shave.blob \
   ~/robot_jazzy_ws/src/balance/models/ball_detector_v3.blob

cp ~/temp/luxonis_export_640/best.json \
   ~/robot_jazzy_ws/src/balance/models/ball_detector_v3.json
```

In the Humble Docker container:
```bash
# The json is auto-loaded if it has the same name as the blob
ros2 run balance ball_detector_oak \
  --ros-args \
  -p blob_path:=/home/ubuntu/robot_ws/src/balance/models/ball_detector_v3.blob \
  -p input_size:=640
```

Verify detection rate:
```bash
ros2 topic hz /ball/position
# Expected: ~20-25Hz at 640x640 (vs 30Hz at 416x416)
```

---

## Phase 9 — Commit to Git

```bash
cd ~/robot_ws/src
git add balance/models/ball_detector_v3.blob
git add balance/models/ball_detector_v3.json
git commit -m "Add 640x640 retrained ball detector with improved camera coverage"
git push
```

---

## Troubleshooting

**Blob conversion fails at 640x640:**
- MyriadX out of memory — try 5 shaves instead of 6 at tools.luxonis.com
- Or fall back to 512x512 as a middle ground

**mAP50 below 0.90 after training:**
- Add more images in the failing cases (check confusion matrix in results)
- Increase epochs to 150
- Check data.yaml paths are correct

**Detection still unstable after retraining:**
- Check the camera image thumbnail in RViz — is the cup fully in frame?
- Increase conf_threshold parameter to 0.6 or 0.7
- Add more negative examples to training data

**MyriadX shows 'Mask not defined' errors:**
- This is expected for YOLOv8 with empty anchor masks
- Detections should still work — check /ball/position topic
- If no detections, the blob may need recompilation
