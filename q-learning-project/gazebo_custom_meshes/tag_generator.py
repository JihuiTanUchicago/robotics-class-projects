import cv2

MARKERS = [1, 2, 3]
if __name__ == "__main__":
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for id in MARKERS:
        marker = cv2.aruco.drawMarker(aruco_dict, id, 64)
        cv2.imwrite(f"marker{id}.png", marker)
