import cv2
import numpy as np
import pyrealsense2 as rs


#Funktioniert aktuell noch nicht ganz, das Alignment zwischen depth_frame und color_frame muss angepasst werden
def visualise_marker(frames, marker_points_image, center_point, camera_extrensics):
    for deviceInfo in frames:
        deviceID = deviceInfo[0]
        frames = frames[deviceInfo]
        color_frame = frames[rs.stream.color].data
        color_image = np.asarray(color_frame)
        infra_frame = frames[rs.stream.infrared,1].data
        infra_image = np.asarray(infra_frame)
        infra_image = cv2.cvtColor(infra_image, cv2.COLOR_GRAY2RGB)
        color = [255,0,0]
        color_center = [0,255,0]



        for point in marker_points_image:
            # Depth-Image Punkte auf RGB-bild mappen
            point_RGB = rs.rs2_transform_point_to_point(camera_extrensics[deviceID], point)
            p_center = (int(point_RGB[0]), int(point_RGB[1]))
            cv2.circle(color_image,p_center,4,color, 5)
            cv2.circle(color_image, p_center, 4, color, 5)
            cv2.circle(infra_image, p_center, 4, color, 5)
            cv2.circle(infra_image, p_center, 4, color, 5)


        center_RGB = rs.rs2_transform_point_to_point(camera_extrensics[deviceID], center_point)
        c_center = (int(center_RGB[0]), int(center_RGB[1]))
        cv2.circle(color_image, c_center, 2 , color_center, 4)
        cv2.circle(infra_image, c_center, 2 , color_center, 4)

        cv2.imshow("Bild der Kamera " + deviceID, color_image)
        cv2.imshow("Bild der Kamera 2 " + deviceID, infra_image)
        cv2.waitKey(1)