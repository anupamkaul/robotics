from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO('yolov8n.pt')

# Run video prediction with Apple Silicon (MPS) acceleration
#results = model.predict(source='./sample-videos/people-detection.mp4', device='mps', show=True, save=True)
results = model.predict(source='./sample-videos/classroom.mp4', device='mps', show=True, save=True)


