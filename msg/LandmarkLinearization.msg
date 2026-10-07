# Linearization point of one SLAM landmark, and the Jacobian of its GLOBAL
# position with respect to its own representation and to the states that
# representation depends on, from OpenVINS's
# UpdaterHelper::get_feature_jacobian_representation (same FEJ choice OpenVINS
# makes). With p_FinG the global position:
#   d p_FinG = H_f * d(rep) + sum_k H_x[k] * d(x_k)
# where d(rep) is this landmark's 3-dof error state (block state_id) and x_k are
# the states listed in hx_state_ids (e.g. the anchor clone, 6 dof JPL [dtheta dp]).
int64 feature_id
int32 state_id               # block of this landmark in the covariance
string representation        # LandmarkRepresentation::as_string
int32 anchor_camera_id       # -1 for global representations
float64 anchor_clone_timestamp
float64[3] p_fing            # global position at the current estimate
float64[3] p_fing_fej        # global position OpenVINS linearizes the measurement at
                             # (equals p_fing for anchored representations, as in
                             # UpdaterHelper::get_feature_jacobian_full)
float64[3] rep_value         # the landmark's own 3-vector (rep coordinates)
float64[9] h_f               # 3x3 row-major d p_FinG / d rep
int32[] hx_state_ids         # state ids of the states in H_x, in order
int32[] hx_sizes             # their error-state sizes
float64[] h_x                # 3 x sum(hx_sizes) row-major, blocks in hx_state_ids order
