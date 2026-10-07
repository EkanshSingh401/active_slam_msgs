# Mean (linearization point) of one OpenVINS pose state: the IMU state's pose
# part, or a sliding-window clone. Conventions are OpenVINS's own, named
# explicitly so nobody has to guess:
#   q_GtoI  JPL quaternion [x y z w] of the rotation global -> IMU
#           (numerically the Hamilton quaternion of R_ItoG)
#   p_IinG  IMU position in the global frame
# *_fej are the first-estimate values OpenVINS linearizes at when use_fej is
# true (state_uses_fej in JointCovariance says which applies).
int32 state_id          # ov_type::Type::id() of the pose; matches StateBlock.state_id
float64 timestamp       # clone timestamp (clones) or state time (IMU)
float64[4] q_gtoi
float64[3] p_iing
float64[4] q_gtoi_fej
float64[3] p_iing_fej
