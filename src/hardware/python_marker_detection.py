import cv2
import cv2.aruco as aruco
import numpy as np
import pandas as pd
import sys

# ==========================================
# CONFIGURATION
# ==========================================
VIDEO_FILENAME = r"C:\Users\visvam\Downloads\en1.mp4"  # <--- REPLACE THIS with your video file name
MARKER_SIZE = 0.10                     # Size of the marker in meters (e.g., 0.10 for 10cm)
ARUCO_DICT_TYPE = aruco.DICT_4X4_50    # Change if your markers use a different dictionary

# Camera Matrix (Ideally, replace with calibration data from the camera that recorded the video)
# These are dummy values for a standard aspect ratio.
CAMERA_MATRIX = np.array([[1000, 0, 640],
                          [0, 1000, 360],
                          [0, 0, 1]], dtype=float)
DIST_COEFFS = np.zeros((5, 1))
# ==========================================

def get_marker_model_points(size):
    """
    Returns the 3D coordinates of the marker corners in the marker's own coordinate system.
    Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
    """
    half_size = size / 2.0
    return np.array([
        [-half_size, half_size, 0],  # Top-Left
        [half_size, half_size, 0],   # Top-Right
        [half_size, -half_size, 0],  # Bottom-Right
        [-half_size, -half_size, 0]  # Bottom-Left
    ], dtype=np.float32)

def get_marker_to_world_matrix(row):
    """
    Constructs the 4x4 transformation matrix from Marker to World using CSV data.
    """
    x, y, z = row['X'], row['Y'], row['Z']
    yaw_deg = row['Yaw_deg']
    yaw = np.radians(yaw_deg)
    
    # Rotation around Z-axis
    cos_a = np.cos(yaw)
    sin_a = np.sin(yaw)
    
    R = np.array([
        [cos_a, -sin_a, 0],
        [sin_a, cos_a, 0],
        [0, 0, 1]
    ])
    
    T_world_marker = np.eye(4)
    T_world_marker[:3, :3] = R
    T_world_marker[:3, 3] = [x, y, z]
    return T_world_marker

def main():
    # 1. Load Map
    try:
        marker_map = pd.read_csv(r"C:\Users\visvam\Downloads\ARUCO.csv")
        print(f"Loaded {len(marker_map)} markers from ARUCO.csv")
    except Exception as e:
        print(f"Error loading ARUCO.csv: {e}")
        return

    # 2. Open Video
    cap = cv2.VideoCapture(VIDEO_FILENAME)
    if not cap.isOpened():
        print(f"Error: Could not open video file '{VIDEO_FILENAME}'. Check the path.")
        return

    # 3. Setup Aruco
    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    parameters = aruco.DetectorParameters()
    
    # Pre-calculate model points for solvePnP
    model_points = get_marker_model_points(MARKER_SIZE)

    print("Processing video... Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of video.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect Markers
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i, marker_id in enumerate(ids.flatten()):
                # Check if this ID exists in our map
                marker_info = marker_map[marker_map['ID'] == marker_id]
                
                if not marker_info.empty:
                    # A. Solve PnP to get Marker Pose in Camera Frame
                    # This replaces estimatePoseSingleMarkers
                    image_points = corners[i][0] # The 4 corners of the current marker
                    success, rvec, tvec = cv2.solvePnP(model_points, image_points, CAMERA_MATRIX, DIST_COEFFS)
                    
                    if success:
                        # B. Calculate Robot Position
                        # T_camera_marker (4x4)
                        R_camera_marker, _ = cv2.Rodrigues(rvec)
                        T_camera_marker = np.eye(4)
                        T_camera_marker[:3, :3] = R_camera_marker
                        T_camera_marker[:3, 3] = tvec.squeeze()

                        # T_world_marker (4x4) from CSV
                        T_world_marker = get_marker_to_world_matrix(marker_info.iloc[0])

                        # T_world_camera = T_world_marker * (T_camera_marker)^-1
                        T_camera_marker_inv = np.linalg.inv(T_camera_marker)
                        T_world_camera = np.dot(T_world_marker, T_camera_marker_inv)

                        # Extract coordinates
                        r_x = T_world_camera[0, 3]
                        r_y = T_world_camera[1, 3]
                        r_z = T_world_camera[2, 3]
                        r_yaw = np.degrees(np.arctan2(T_world_camera[1, 0], T_world_camera[0, 0]))

                        # Draw Text and Axis
                        text = f"ID:{marker_id} Robot:({r_x:.2f}, {r_y:.2f})"
                        cv2.putText(frame, text, (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.drawFrameAxes(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, 0.05)

        cv2.imshow('Robot Position from Video', frame)
        
        # Wait 25ms (approx 40fps) or quit on 'q'
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if _name_ == "_main_":
    main()
