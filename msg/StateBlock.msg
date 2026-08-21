# Describes one variable block inside a JointCovariance matrix.
#
# The joint covariance is published as a dense row-major matrix. This message
# says what each diagonal block IS, so a consumer can slice out the block it
# wants (and the cross-covariance between any two blocks) without ever
# hardcoding an offset.
#
# IMPORTANT: OpenVINS re-indexes its covariance whenever a clone or SLAM
# feature is marginalized -- the matrix physically shrinks and every id shifts.
# Never cache these offsets across messages. Re-read them every time.

# --- what kind of variable this is ---
string TYPE_IMU=imu                      # 15x15: JPL quat(3) pos(3) vel(3) bg(3) ba(3)
string TYPE_CLONE=clone                  # 6x6: JPL quat(3) pos(3) of a sliding-window pose
string TYPE_SLAM_FEATURE=slam_feature    # 3x3: see parameterization field
string TYPE_CALIB_DT=calib_dt            # 1x1: cam-imu time offset
string TYPE_CAM_EXTRINSICS=cam_extrinsics    # 6x6: per-camera T_ItoC
string TYPE_CAM_INTRINSICS=cam_intrinsics    # 8x8: fx fy cx cy + 4 distortion
string TYPE_IMU_INTRINSICS=imu_intrinsics    # dw/da/tg/GYROtoIMU/ACCtoIMU
string type

# --- where this block lives IN THIS MESSAGE's covariance matrix ---
int32 index      # start row/col within JointCovariance.covariance
int32 size       # block is size x size on the diagonal

# --- where it lives in the estimator's own covariance (diagnostic) ---
int32 state_id   # ov_type::Type::id(), i.e. offset into State::_Cov

# --- optional identity fields, meaningful per type ---
float64 clone_timestamp   # TYPE_CLONE: timestamp of this sliding-window pose
int64 feature_id          # TYPE_SLAM_FEATURE: OpenVINS landmark id
int32 camera_id           # TYPE_CAM_*: which camera

# --- SLAM feature parameterization (THIS MATTERS) ---
# For TYPE_SLAM_FEATURE this is the OpenVINS LandmarkRepresentation as a
# string, e.g. "ANCHORED_MSCKF_INVERSE_DEPTH" or "GLOBAL_3D".
#
# If it is an ANCHORED_* representation the 3-vector is NOT global XYZ: it is
# expressed relative to the anchor clone identified below. Converting this
# block (and its cross-covariance with the IMU) into metric world-frame
# uncertainty requires propagating through the anchored->XYZ Jacobian, which
# also depends on the anchor clone pose. The anchor clone's own block and its
# cross-terms are present in the same matrix, so the conversion is possible
# entirely from one message -- but it is deliberately NOT done here.
string parameterization
bool is_anchored
int32 anchor_camera_id
float64 anchor_clone_timestamp
