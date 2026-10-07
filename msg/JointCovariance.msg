# Joint (not marginal) covariance over a selected set of OpenVINS state
# variables, with the structural metadata needed to interpret it.
#
# This is the whole point of the message: stock OpenVINS publishes only the
# 6x6 marginal pose covariance on /odomimu. A Fisher-information planner needs
# the joint pose+map block INCLUDING the IMU<->feature cross-covariance, which
# is what this carries.
#
# The matrix is dense, symmetric, row-major, dim x dim. Blocks appear in the
# same order they were packed, and `blocks` tells you what each one is.
#
# To get the cross-covariance between block i and block j:
#   rows blocks[i].index .. +blocks[i].size
#   cols blocks[j].index .. +blocks[j].size

std_msgs/Header header      # stamp matches the OpenVINS state timestamp

int32 dim                   # covariance is dim x dim
float64[] covariance        # row-major, length dim*dim

StateBlock[] blocks         # what each block is; sum(blocks[].size) == dim

# --- context / provenance ---
int32 full_state_dim        # State::max_covariance_size(): size of the FULL
                            # estimator covariance. If dim < full_state_dim,
                            # some variables were deliberately excluded (see
                            # the include_* flags below) -- the published
                            # matrix is then the joint covariance of the
                            # included subset, which for a Gaussian is exactly
                            # its marginal, with all internal cross-terms kept.
bool includes_imu
bool includes_clones
bool includes_slam_features
bool includes_calibration

int32 num_clones
int32 num_slam_features

# --- linearization point (PATCHES s60) ---
# The means the covariance above is about, so a consumer can build measurement
# Jacobians at the estimator's own linearization point from one message.
bool state_uses_fej            # OpenVINS use_fej: Jacobians are taken at *_fej
PoseLinearization imu_pose
PoseLinearization[] clone_poses        # same order as the clone blocks
LandmarkLinearization[] landmarks      # same order as the slam_feature blocks
CameraCalibration[] cameras
