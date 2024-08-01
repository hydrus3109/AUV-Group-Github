import numpy
import ultralytics
from ultralytics import YOLO
ultralytics.checks()
def init(self):
    self.model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)

def predict(self, img):
    results = self.model(img)  # return a list of Results objects
    boxes = results[0].boxes
    x1, y1, x2, y2 = boxes.xyxy