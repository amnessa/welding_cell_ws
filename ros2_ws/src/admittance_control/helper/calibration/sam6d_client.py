import time
import json
import requests
import numpy as np
import cv2
import pyrealsense2 as rs

# SAM-6D Server Address (Because both containers use --network host, localhost works)
SERVER_URL = 'http://127.0.0.1:5000/predict_pose'

def get_camera_intrinsics(profile):
    """Extract intrinsics and format them for SAM-6D."""
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    color_stream = profile.get_stream(rs.stream.color)
    intr = color_stream.as_video_stream_profile().get_intrinsics()

    return {
        "cam_K": [intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0],
        "depth_scale": depth_scale
    }

def main():
    print("Starting RealSense pipeline...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

    # Let the camera auto-expose for a second
    time.sleep(1.0)

    try:
        print("Capturing frames...")
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            raise RuntimeError("Could not capture frames.")

        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Generate the camera.json dictionary
        camera_data = get_camera_intrinsics(profile)

        print("Encoding data for transmission...")
        # Encode images to memory buffers (PNG format)
        _, rgb_encoded = cv2.imencode('.png', color_image)
        _, depth_encoded = cv2.imencode('.png', depth_image)
        camera_encoded = json.dumps(camera_data).encode('utf-8')

        # Package the payload
        files = {
            'rgb': ('rgb.png', rgb_encoded.tobytes(), 'image/png'),
            'depth': ('depth.png', depth_encoded.tobytes(), 'image/png'),
            'camera': ('camera.json', camera_encoded, 'application/json')
        }

        print("Sending request to SAM-6D server...")
        response = requests.post(SERVER_URL, files=files)

        if response.status_code == 200:
            print("Success! Response from server:")
            print(json.dumps(response.json(), indent=4))
        else:
            print(f"Server Error {response.status_code}: {response.text}")

    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()