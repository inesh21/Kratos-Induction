import cv2
import numpy as np


class ArucoDetector:

    def __init__(self, aruco_dict=cv2.aruco.DICT_4X4_1000):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)

    def detect_markers(self, image):
        corners, ids, rejected = self.detector.detectMarkers(image)
        # No marker found
        if ids is None:
            print("No marker detected, rejected corners:", len(rejected))
            return None

        # First detected marker
        marker_id = int(ids[0][0])
        marker_corners = corners[0]

        # Calculate marker center
        center = np.mean(marker_corners[0], axis=0)
        center_x = int(center[0])
        center_y = int(center[1])

        # Calculate marker width (top edge)
        top_left = marker_corners[0][0]
        top_right = marker_corners[0][1]
        width = np.linalg.norm(top_right - top_left)

        return {
            "id": marker_id,
            "center_x": center_x,
            "center_y": center_y,
            "width": width,
            "corners": marker_corners
        }