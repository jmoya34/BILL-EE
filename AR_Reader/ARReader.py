import cv2
import cv2.aruco as aruco


def findArucoMarkers(img, markerSize=4, totalMarker=250, draw=True):
    """
    :param img: image to detect AR tags
    :param markerSize: the size of the markers
    :param totalMarker: total number of markers that compose the dictionary
    :param draw: flag to draw bounding box around markers detective
    :return: bounding boxes and ID numbers of AR detected
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarker}')
    arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_250)

    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img, bboxs)

    return [bboxs, ids]


def main():
    aruco_tags = ['Start', 'Post_1', 'Post_2', 'Post_3', 'Gate_1', 'Gate_2']

    cap = cv2.VideoCapture(0)
    while True:
        success, img = cap.read()
        arucoFound = findArucoMarkers(img)

        # Loop through all the markers
        if len(arucoFound[0]) != 0:
            for bbox, id in zip(arucoFound[0], arucoFound[1]):
                if 0 <= int(id) <= 5:
                    x, y, _, _ = cv2.boundingRect(bbox) # coordinate for the bounding boxes of each AR tag
                    cv2.putText(img, str(aruco_tags[int(id)]), (x, y-10), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)

        cv2.imshow('Cam', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    main()
