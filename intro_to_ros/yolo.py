import numpy
import ultralytics
from ultralytics import YOLO
import cv2
import numpy as np
import matplotlib.pyplot as plt

ultralytics.checks()
def init():
    model = YOLO("intro_to_ros/best_ncnn_model")  
    return model# load a pretrained model (recommended for training)

def predict(self, img):
    results = self.model(img)  # return a list of Results objects
    boxes = results[0].boxes
    x1, y1, x2, y2 = boxes.xyxy

def predict(model, image):
    if image is None:
        print("None")
    else:
        print("not none")
        imgwidth = np.shape(image)[1]
        imgheight = np.shape(image)[0]
        image = image[int(imgheight*0.20):imgheight, 0:imgwidth]

        results = model(image)
        boxes = results[0].boxes
        conf = boxes.conf.numpy().squeeze()
        print(conf)
        confidx = 0
        bestconf = 0
        for i in range(len(conf)):
            if conf[i] > bestconf:
                bestconf = conf[i]
                confidx = i
        coords= boxes.xyxy.numpy().squeeze()[confidx]
        if coords is not None:
            print(coords)
            annotated_frame = results[0].plot()
            xcenter =(coords[0]+coords[2])/2
            print(xcenter)
            plt.imsave("intro_to_ros/yolo_imgsave.png",annotated_frame)
            return coords, xcenter
def test_func():
    #image = cv2.imread('/home/kenayosh/auvc_ws/src/AUV-Group-Github/intro_to_ros/images/apriltag.png') #This is actaully the pool image
    image = cv2.imread('/home/kenayosh/auvc_ws/src/AUV-Group-Github/intro_to_ros/images/dock AUV.png')
    #image = cv2.imread('/home/kenayosh/auvc_ws/src/AUV-Group-Github/intro_to_ros/images/Screenshot 2024-08-01 235439.png')
    plt.imsave("intro_to_ros/yolo_imgsave.png",image)
    
    if image is None:
        print("None")
    else:
        print("not none")
        imgwidth = np.shape(image)[1]
        imgheight = np.shape(image)[0]
        image = image[int(imgheight*0.20):imgheight, 0:imgwidth]
        model = YOLO("intro_to_ros/best_ncnn_model")
        results = model(image)
        boxes = results[0].boxes
        conf = boxes.conf.numpy().squeeze()
        print(conf)
        confidx = 0
        bestconf = 0
        for i in range(len(conf)):
            if conf[i] > bestconf:
                bestconf = conf[i]
                confidx = i
        coords= boxes.xyxy.numpy().squeeze()[confidx]
        if coords is not None:
            print(coords)
            annotated_frame = results[0].plot()
            xcenter =(coords[0]+coords[2])/2
            print(xcenter)
            plt.imsave("intro_to_ros/yolo_imgsave.png",annotated_frame)
            return coords, xcenter
    
test_func()