


import re
import cv2

def show_detected_objects():
    """
    This function reads a text file containing information about detected objects,
    loads corresponding images, and draws bounding boxes around the detected objects.
    It then displays the images with the bounding boxes.
    """
    # Open the text file
    with open('detected_objects.txt', 'r') as f:
        lines = f.readlines()

    # Initialize variables
    img = None
    filename = ""            
    for line in lines:
        if "Image:" in line:
            # Save previous image
            if img is not None:
                cv2.imshow("Image", img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            # Load new image
            filename = line.split(": ")[1].strip()
            img = cv2.imread('images/' + filename)
        elif "Bounding Box:" in line:
            # Extract bounding box coordinates
            bbox = re.findall(r'\d+', line)
            x, y, w, h = map(int, bbox)

            # Draw rectangle around detected object
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Display last image
    if img is not None:
        cv2.imshow("Image", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
