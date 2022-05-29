import time

import numpy
import numpy as np
import cv2

import pyrealsense2 as rs
import sklearn
from sklearn.cluster import DBSCAN,KMeans


def find_markers_for_cam(frame, intrinsics, device_transformation):
    marker_points = np.empty((0, 2))
    depth_frame = frame[rs.stream.depth].data
    infrared_frame = frame[rs.stream.infrared, 1].data
    depth_intrinsics = intrinsics[rs.stream.depth]

    # Umwandlung der Streams in opencv-Formate
    infrared_image = np.asarray(infrared_frame)
    depth_image = np.asarray(depth_frame)

    # selektion der reflectierenden Marker
    # Andere Threshold Varianten auch möglich, z.B. otsu
    threshold_value = 160
    result, infraredFrameTresholt = cv2.threshold(infrared_image, threshold_value, 255, cv2.THRESH_BINARY)

    #Selektierte Stellen als Marker identifizieren
    contours, hierarchy = cv2.findContours(infraredFrameTresholt, cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)


    # cv2.imshow("infrared", infrared_image)
    cv2.imshow("thresh", infraredFrameTresholt)
    cv2.waitKey(0)
    # cv2.destroyAllWindows()
    for cnt in contours:
        # Moment der Marker berechnen
        # moment = gewichtete Mittelwerte der contur
        moment = cv2.moments(cnt)
        if moment["m00"] != 0:
            x = int(moment["m10"] / moment["m00"])
            y = int(moment["m01"] / moment["m00"])
            point = np.array([x,y])
            marker_points = np.vstack((marker_points, point))
    #es ist möglich dass mehrere IR-Punkte pro ""markerkugel" gefudnen werden. Mit Kmeans die Kugeln clustern
    #marker_count = anzahl der "kugeln" auf dem marker
    #wenn Anzahl erkannter konturen < 3 nicht möglich, Versuch wird abgebrochen -> Stabilität in der Verfolgung
    marker_count = 3
    if (len(marker_points) < marker_count):
        print("Marker wurde nicht erkannt")
        print("Erkennung wird abgebrochen")
        #das ist nicht schön hihi
        return False, [0],[0]
    marker_cluster = cluster_KMEANS(marker_count, marker_points)

    #Markercentren gefunden
    points_world = np.empty([0, 3])
    points_image = np.empty([0, 3])
    # aus 2D-Bildkoordinaten 3D-Weltkoordinaten berechnen
    for point in marker_cluster.cluster_centers_:
        xB = int(point[0])
        yB = int(point[1])
        zB = depth_image[yB, xB]
        xW = (int(point[0]) - depth_intrinsics.ppx) / depth_intrinsics.fx
        yW = (int(point[1]) - depth_intrinsics.ppy) / depth_intrinsics.fy
        zW = depth_image[yB, xB] / 1000    #default depth scale für D435 is 1:1000
        depth = np.array([zW*xW, zW*yW, zW, 1])
        # mithilfe der Transformationsmatrix die Echtwelt-koordinaten errechnen
        depth = np.array(np.matmul(device_transformation.pose_mat, depth)[0:3])
        points_world = np.vstack((points_world, depth))
        plane = np.array([xB,yB,zB])
        points_image = np.vstack((points_image, plane))

    return True, points_image, points_world



def find_markers_all_cams(frames_devices, intrinsics_devices, device_transformation):

    #Marker und dazugeörige Kamera werden hier gespeichert
    found_markers_world = np.empty((0,3))
    found_markers_image = np.empty((0,3))
    markers_found_all_cams = False;

    for deviceInfo in frames_devices:
        deviceID = deviceInfo[0]
        frames = frames_devices[deviceInfo]
        intrinsics = intrinsics_devices[deviceID]
        transformation = device_transformation[deviceID][0] #nur die Transformationsmatrix

        markers_found, points_image, points_world = find_markers_for_cam(frames, intrinsics, transformation)
        if(markers_found):
            found_markers_world = np.append(found_markers_world, points_world, axis=0)
            found_markers_image = np.append(found_markers_image, points_image, axis=0)
            markers_found_all_cams = True


    # hier Markenclustering einführen
    # zum Beispiel mt DBSCAN um Noise herauszufiltern
    #für alle Kameras
    #DBSCAN(eps=<0.07,points,1)
    #findet alle Marker, die von mehreren Kameras gesehen werden

    return markers_found_all_cams, found_markers_image, found_markers_world



#DBSCAN Clustering
def cluster_DBSCAN(eps_distance, min_samples, points):
    cluster = DBSCAN(eps=eps_distance, min_samples=min_samples).fit(points)


#KMEANS Clustering
def cluster_KMEANS(cluster_count, points):
    cluster = KMeans(n_clusters=cluster_count, random_state=0).fit(points)
    return cluster


def convert_2D_to_3D_WorldCoordinate(point, depth_value, depth_intrinsics, device_transformation):
    points_3D = np.empty([0, 3])
    x1 = (int(point[0]) - depth_intrinsics.ppx) / depth_intrinsics.fx
    y1 = (int(point[1]) - depth_intrinsics.ppy) / depth_intrinsics.fy
    z = depth_value / 1000    #default depth scale for D435 is 1:1000
    depth = np.array([z*x1, z*y1, z, 1])
    # mithilfe der Transformationsmatrix die Echtwelt-koordinaten errechnen
    depth = np.array(np.matmul(device_transformation.pose_mat, depth)[0:3])
    points_3D = np.vstack((points_3D, depth))

    #Punkte in 3D-Welt-Koordinaten
    return points_3D


#calculates the Center of 3D-Points
def calculate_center_3D(points):
    x = 0
    y = 0
    z = 0
    for p in points:
        x = x + p[0]
        y = y + p[1]
        z = z + p[2]
    xc = x / len(points)
    yc = y / len(points)
    zc = z / len(points)

    center = numpy.array([xc,yc,zc])
    return center
