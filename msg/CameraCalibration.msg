# One camera's calibration as OpenVINS currently holds it (estimated or fixed).
#   q_itoc  JPL quaternion [x y z w] of the rotation IMU -> camera
#   p_iinc  IMU origin in the camera frame
#   intrinsics  OpenVINS order: fx fy cx cy d1 d2 d3 d4
# state ids are -1 when the quantity is not being estimated (not in the covariance).
int32 camera_id
string model                 # "radtan" or "equidistant"
float64[4] q_itoc
float64[3] p_iinc
float64[8] intrinsics
int32 extrinsics_state_id
int32 intrinsics_state_id
