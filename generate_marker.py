from pathlib import Path
import cv2
import numpy as np

OUT = Path(__file__).resolve().parent / "assets" / "aruco_0.png"
OUT.parent.mkdir(parents=True, exist_ok=True)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker = cv2.aruco.generateImageMarker(dictionary, 0, 700)
canvas = np.full((980, 980), 255, dtype=np.uint8)
canvas[140:840, 140:840] = marker
cv2.imwrite(str(OUT), canvas)
print(f"Wrote {OUT}")
