import pickle
import time

import cv2
import pyrealsense2 as rs

import device_manager
from helper_calibration_kabsch import PoseEstimation
import numpy as np

# Calibration the devices to world coordinates using a printed out black and white chessboard and the Kabsch Estimation

# set the chessboard params
chessboard_width = 6  # squares
chessboard_height = 9  # squares
square_size = 0.0253  # meters


#returns frames after the cameras have stabilised
def get_stable_Frames(device_manager):
    waiting_frames = 30
    for frame in range(waiting_frames):
        device_manager.poll_frames()

    frames = device_manager.poll_frames()
    return frames


def save_calibration(device_transformation):
    file = open("calibration/camera_calibration", 'wb')
    pickle.dump(device_transformation, file)

def load_calibration():
    file = open("calibration/camera_calibration", 'rb')
    device_transformation = pickle.load(file)
    return device_transformation


def calibrate_all_devices(device_manager):

    chessboard_params=[chessboard_height, chessboard_width, square_size]
    #Presetzum erkennen des Schachbretts
    device_manager.load_settings_json("presets/preset_HighResHighAccuracy.json")

    #Warten bis der Autofokus stabilisiert ist
    frames = get_stable_Frames(device_manager)

    intrinsics_devices = device_manager.get_device_intrinsics(frames)

    for device in device_manager._available_devices:
        calibrated_device_count = 0
        while calibrated_device_count < len(device_manager._available_devices):
            frames = device_manager.poll_frames()
            pose_estimator = PoseEstimation(frames, intrinsics_devices, chessboard_params)
            transformation_result_kabsch = pose_estimator.perform_pose_estimation()
            calibrated_device_count = 0
            for device_info in device_manager._available_devices:
                deviceID = device_info[0]
                if not transformation_result_kabsch[deviceID][0]:
                    infra_frame = frames[device][rs.stream.infrared, 1].data
                    infra_image = np.asarray((infra_frame))
                    print("Schachbrett in Sichtfeld der Kamera positionieren")
                    cv2.imshow("Sichtfeld", infra_image)
                    cv2.waitKey(1)

                else:
                    calibrated_device_count += 1

    device_Transformation = {}
    for device_info in device_manager._available_devices:
        deviceID = device_info[0] #serial-ID of device
        transformation_matrix = transformation_result_kabsch[deviceID][1].inverse()   # 4x4 Transformation Matrix
        kabsch_rmsd = transformation_result_kabsch[deviceID][3]  # rmsd
        device_Transformation[deviceID] = (transformation_matrix, kabsch_rmsd)

    return device_Transformation