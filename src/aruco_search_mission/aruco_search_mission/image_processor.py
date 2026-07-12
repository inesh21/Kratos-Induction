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

        return ids, corners