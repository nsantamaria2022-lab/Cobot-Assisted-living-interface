#best_fcnn_model.pth and category_ids.txt should be in the same folder with this
import os
import cv2
import torch
import numpy as np
from PIL import Image
import torch.nn as nn
from torchvision import transforms

# ----------------------------
# Paths (same folder as this script)
# ----------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
WEIGHTS_PATH  = os.path.join(HERE, "best_fcnn_model.pth")
CATEGORY_FILE = os.path.join(HERE, "category_ids.txt")

IMG_SIZE = 128
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
THRESH = 0.70
MIN_BOX_AREA = 0.02
# ----------------------------
# Load categories
# ----------------------------
id_to_class = {}
with open(CATEGORY_FILE, "r") as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) != 2:
            continue
        a, b = parts
        try:
            idx = int(a)
            label = b
        except ValueError:
            label = a
            idx = int(b)
        id_to_class[idx] = label

NUM_CLASSES = len(id_to_class)
print(f"Loaded {NUM_CLASSES} classes")

# ----------------------------
# Model (must match training exactly)
# ----------------------------
class FCNNMultiTask(nn.Module):
    def __init__(self, num_classes):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 32, 3, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(32, 64, 3, padding=1)
        self.conv3 = nn.Conv2d(64, 128, 3, padding=1)
        self.fc1 = nn.Linear((IMG_SIZE // 8) * (IMG_SIZE // 8) * 128, 256)
        self.dropout = nn.Dropout(0.4)
        self.fc2 = nn.Linear(256, 128)
        self.class_head = nn.Linear(128, num_classes)
        self.box_head = nn.Linear(128, 4)

    def forward(self, x):
        x = self.pool(torch.relu(self.conv1(x)))
        x = self.pool(torch.relu(self.conv2(x)))
        x = self.pool(torch.relu(self.conv3(x)))
        x = x.view(x.size(0), -1)
        x = torch.relu(self.fc1(x))
        x = self.dropout(x)
        x = torch.relu(self.fc2(x))
        class_out = self.class_head(x)
        box_out = torch.sigmoid(self.box_head(x))  # normalized [0..1]
        return class_out, box_out

model = FCNNMultiTask(NUM_CLASSES).to(DEVICE)
state = torch.load(WEIGHTS_PATH, map_location=DEVICE)
model.load_state_dict(state)
model.eval()
print("Model loaded.")

# ----------------------------
# Preprocess (must match your training)
# ----------------------------
preprocess = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225])
])

def predict(frame_bgr):
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)

    x = preprocess(pil).unsqueeze(0).to(DEVICE)

    with torch.no_grad():
        cls_out, box_out = model(x)
        pred_id = int(torch.argmax(cls_out, dim=1).item())
        prob = torch.softmax(cls_out, dim=1).max().item()
        box = box_out.squeeze(0).cpu().numpy()

    return pred_id, prob, box

# ----------------------------
# Webcam loop
# ----------------------------
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam (VideoCapture(0)). Try index 1.")

print("Press 'q' to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        break

    pred_id, prob, box = predict(frame)

    label = id_to_class.get(pred_id, str(pred_id))

    H, W = frame.shape[:2]
    x1, y1, x2, y2 = (box * np.array([W, H, W, H])).astype(int)

    # clamp + sort
    x1, x2 = sorted([max(0, min(W-1, x1)), max(0, min(W-1, x2))])
    y1, y2 = sorted([max(0, min(H-1, y1)), max(0, min(H-1, y2))])

    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(frame, f"{label} ({prob:.2f})", (x1, max(0, y1-10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Food Recognition", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
