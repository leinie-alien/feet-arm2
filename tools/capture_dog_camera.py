#!/usr/bin/env python3
import cv2
import sys

device = int(sys.argv[1]) if len(sys.argv) > 1 else 8
out_dir = sys.argv[2] if len(sys.argv) > 2 else "recordings/dog_camera"

cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not cap.isOpened():
    print(f"Cannot open /dev/video{device}")
    sys.exit(1)

i = 0
while True:
    try:
        input(f"Press Enter to capture frame {i} (Ctrl+C to stop)...")
    except KeyboardInterrupt:
        print("\nDone.")
        break
    ret, frame = cap.read()
    if not ret:
        print("failed to read frame")
        continue
    path = f"{out_dir}/dog_{i:04d}.png"
    cv2.imwrite(path, frame)
    print(f"saved {path}")
    i += 1

cap.release()
