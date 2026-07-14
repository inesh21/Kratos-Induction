import cv2
import numpy as np


class ArucoDetector:

    def __init__(self):

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_1000
        )

        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            self.parameters
        )

    def detect(self, frame):

        corners, ids, rejected = self.detector.detectMarkers(frame)

        if ids is None or len(ids) == 0:
            return None

        pts = corners[0][0]

        center_x = np.mean(pts[:, 0])
        center_y = np.mean(pts[:, 1])
        marker_width = np.linalg.norm(pts[0] - pts[1])

        return {
            "id": int(ids[0][0]),
            "center_x": center_x,
            "center_y": center_y,
            "marker_width": marker_width,
            "corners": pts,
        }