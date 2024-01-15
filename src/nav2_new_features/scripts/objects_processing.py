"""Image: [image filename]
Object: [object name]
Confidence: [confidence score]
Bounding Box: [bounding box coordinates]

Image: [image filename]
Object: [object name]
Confidence: [confidence score]
Bounding Box: [bounding box coordinates]
"""
import cv2
import numpy as np
import os



# Load Yolo
net = cv2.dnn.readNet("scripts/yolov3.weights", "scripts/yolov3.cfg")

layer_names = net.getLayerNames()

unconnected_out_layers = net.getUnconnectedOutLayers()
if unconnected_out_layers.ndim == 1:
    output_layers = [layer_names[i - 1] for i in unconnected_out_layers]
else:
    output_layers = [layer_names[i[0] - 1] for i in unconnected_out_layers]

# Load names of classes from coco.names file
classes = []
with open("scripts/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Process all images in the directory
for filename in os.listdir('images'):
    if filename.endswith(".png"):
        img = cv2.imread('images/' + filename)
        height, width, channels = img.shape

        # Detect objects
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Write information to text file
        with open('detected_objects.txt', 'a') as f:
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        # Object detected
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)

                        f.write(f"Image: {filename}\n")
                        f.write(f"Object: {classes[class_id]}\n")
                        f.write(f"Confidence: {confidence}\n")
                        f.write(f"Bounding Box: ({x}, {y}, {w}, {h})\n\n")
                        
import cv2
import numpy as np

# Load the depth map and the color image
depth_map = cv2.imread('depth_map.png', cv2.IMREAD_UNCHANGED)
color_image = cv2.imread('color_image.png')

# Detect the object in the color image
# (This is a placeholder - replace this with your actual object detection code)
x, y, w, h = cv2.boundingRect(color_image)

# Get the depth value from the depth map
depth = depth_map[y+h//2, x+w//2]

# Assume the camera parameters are known
fx = fy = 525  # Focal length
cx = 319.5  # Principal point (x-coordinate)
cy = 239.5  # Principal point (y-coordinate)

# Back-project to 3D
z = depth  # Depth value
x = (x - cx) * z / fx  # X-coordinate
y = (y - cy) * z / fy  # Y-coordinate

print(f"The 3D position of the object is ({x}, {y}, {z})")