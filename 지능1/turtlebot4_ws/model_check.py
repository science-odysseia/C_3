from ultralytics import YOLO

model = YOLO("best-v8n.pt")

print(model.names)