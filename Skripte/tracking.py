import time

import pyrealsense2 as rs
import numpy as np

from device_manager import DeviceManager
from calibrate_cams import calibrate_all_devices, load_calibration, save_calibration
from dedect_markers import find_markers_all_cams, cluster_DBSCAN, cluster_KMEANS, calculate_center_3D
from visualise import visualise_marker


def enable_devices():

	# Constants for different camerastreams
	resolution_depth_width = 1280  # pixels
	resolution_depth_height = 720  # pixels
	depth_frame_rate = 30  # fps

	resolution_color_width = 1280  # pixels
	resolution_color_height = 720  # pixels
	color_frame_rate = 30  # fps

	try:
		# enable streams
		rs_config = rs.config()
		rs_config.enable_stream(rs.stream.depth, resolution_depth_width, resolution_depth_height, rs.format.z16,
								depth_frame_rate)
		rs_config.enable_stream(rs.stream.infrared, 1, resolution_depth_width, resolution_depth_height,
								rs.format.y8, depth_frame_rate)
		rs_config.enable_stream(rs.stream.color, resolution_color_width, resolution_color_height, rs.format.bgr8,
								color_frame_rate)

		# Use the device manager class to enable the devices and get the frames
		device_manager = DeviceManager(rs.context(), rs_config)
		device_manager.enable_all_devices()
		device_manager.enable_emitter(True)

		assert (len(device_manager._available_devices) > 0)

		return device_manager

	finally:
		device_manager.disable_streams()



def get_intrinsics(device_manager):
	frames = device_manager.poll_frames()
	device_intrinsics = device_manager.get_device_intrinsics(frames)
	return device_intrinsics



def track_markers(device_manager, device_transformation):
	#Speichern der Bewegungsvectoren des Markerpunktes (in Meter)
	movement_vectors = np.zeros(3)
	center_world = np.zeros(3)
	center_image = np.zeros(3)
	#pause zwischen Trackingframes 30 = 1sec
	frames_between_tracking = 5
	#Richtige Konfig zum erkennen der Marker laden
	device_manager.load_settings_json('presets/preset_settings_marker_detection.json')
	devices_intrinsics = get_intrinsics(device_manager)

	#Extrensics wird für darstellung benutzt
	frame = device_manager.poll_frames()
	devices_extrinsics = device_manager.get_depth_to_color_extrinsics(frame)

	# Tracking i mal durchführen ; alternativ while True
	#for i in range(0,20):
	while True:
		frames = device_manager.poll_frames()
		markers_found, points_image, points_world  = find_markers_all_cams(frames, devices_intrinsics, device_transformation)
		#marker wurde nicht gefunden
		if not (markers_found):
			print("Marker nicht identifiziert")
			print("erneuter Versuch")
			#i = i-1
		else:
			oldcenter = center_world
			center_world = calculate_center_3D(points_world)
			center_image = calculate_center_3D(points_image)
			movement_world = center_world - oldcenter
			movement_vectors = np.vstack((movement_vectors, movement_world))
			print("Markermittelpunkt:")
			print(center_world)

			visualise_marker(frames, points_image, center_image, devices_extrinsics)
		#"sleeping"69
		for f in range(frames_between_tracking):
			device_manager.poll_frames()

	return movement_vectors



if __name__ == "__main__":

	device_manager = enable_devices()

	device_transformation = calibrate_all_devices(device_manager)
	print("Kalibrierung abgeschlossen")
	#save_calibration(device_transformation)

	#device_transformation = load_calibration()
	#print("Kalibrierung aus Datei geladen")

	#track_markers(device_manager,device_transformation)

	#device_manager.disable_streams()