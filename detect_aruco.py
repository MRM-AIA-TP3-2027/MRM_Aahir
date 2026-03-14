import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# approximate camera matrix (works for demo)
camera_matrix = np.array([[1000, 0, 640],
                          [0, 1000, 360],
                          [0, 0, 1]], dtype=float)

dist_coeffs = np.zeros((5,1))

marker_size = 0.05   # meters (5 cm marker)

# dictionaries to detect
dict_types = [
    cv2.aruco.DICT_4X4_50,
    cv2.aruco.DICT_5X5_50,
    cv2.aruco.DICT_6X6_50
]

detectors = []
for d in dict_types:
    dictionary = cv2.aruco.getPredefinedDictionary(d)
    parameters = cv2.aruco.DetectorParameters()
    detectors.append(cv2.aruco.ArucoDetector(dictionary, parameters))

while True:

    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    for detector in detectors:

        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size, camera_matrix, dist_coeffs
            )

            for i in range(len(ids)):

                cv2.drawFrameAxes(
                    frame,
                    camera_matrix,
                    dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    0.03
                )

                distance = np.linalg.norm(tvecs[i])*0.25

                cv2.putText(
                    frame,
                    f"ID:{ids[i][0]} Dist:{distance:.2f}m",
                    (10, 40 + i*30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0,255,0),
                    2
                )

    cv2.imshow("ArUco Pose Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
