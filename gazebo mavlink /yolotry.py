from ultralytics import YOLO
import cv2

model = YOLO("/home/halit/Desktop/face_detection/runs/detect/train2/weights/best.pt")
results = model.predict(source="test.jpg", show=True)

# Pencerenin q'ya basılana kadar açık kalmasını sağla
cv2.waitKey(0)
cv2.destroyAllWindows()

print(results)
