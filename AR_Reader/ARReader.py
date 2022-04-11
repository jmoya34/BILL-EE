import cv2
import cv2.aruco as aruco
import numpy as np
import math


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def findArucoMarkers(img, draw=True):
    """
    :param img: image to detect AR tags
    :param markerSize: the size of the markers
    :param totalMarker: total number of markers that compose the dictionary
    :param draw: flag to draw bounding box around markers detective
    :return: bounding boxes and ID numbers of AR detected
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_250)

    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img, bboxs)

    return [bboxs, ids]

# --- 180 deg rotation matrix around the x axis
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

def main():
    aruco_tags = ['Start', 'Post_1', 'Post_2', 'Post_3', 'Gate_1', 'Gate_2']
    center_points_cur_frame = []
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    while True:
        ret, img = cap.read()
        arucoFound = findArucoMarkers(img)

        # Loop through all the markers
        if len(arucoFound[0]) != 0:
            for bbox, id in zip(arucoFound[0], arucoFound[1]):
                if 0 <= int(id) <= 5:
                    x, y, w, h = cv2.boundingRect(bbox)  # coordinate for the bounding boxes of each AR tag
                    camera_matrix = np.mat([[1.019099074177694320e+03, 0.0, 6.557727729771451095e+02],
                                            [0.0, 1.011927236550148677e+03, 3.816077913964442700e+02],
                                            [0.0, 0.0, 1.0]])
                    camera_distortion = np.mat([2.576784605153304430e-01, -1.300640184051879311e+00,
                                                -4.285777480424158084e-03, -2.507657388926626523e-03,
                                                2.307018624520866812e+00])
                    ret = aruco.estimatePoseSingleMarkers(bbox, 10, camera_matrix, camera_distortion)
                    rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

                    cv2.putText(img, str(aruco_tags[int(id)]), (x, y - 10), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)

                    # Font for the text in the image
                    font = cv2.FONT_HERSHEY_PLAIN
                    # Draw Axis
                    if np.all(id is not None):
                        for i in range(0, len(id)):
                            aruco.drawAxis(img, camera_matrix, camera_distortion, rvec, tvec, 10)

                            # Display the AR Tag position
                            str_position = "AR Tag Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
                            cv2.putText(img, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow('Cam', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    main()
