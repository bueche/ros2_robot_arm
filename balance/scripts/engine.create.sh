cd ~/temp
python3 - <<'EOF'
from ultralytics import YOLO
model = YOLO('runs/detect/ball_training/run2/weights/best.pt')
model.export(
    format='engine',
    imgsz=416,
    half=True,      # FP16 for Orin GPU
    device=0,
    batch=1
)
print("Done — engine saved alongside best.pt")
EOF
