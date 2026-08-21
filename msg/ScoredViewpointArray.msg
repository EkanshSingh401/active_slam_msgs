# A batch of scored candidate viewpoints.
#
# scoring_method / is_placeholder exist so a consumer (and a human reading
# `ros2 topic echo`) can tell at a glance whether these scores mean anything
# yet. While the real Fisher formulation is unimplemented, is_placeholder is
# true and scoring_method names the stand-in heuristic.

std_msgs/Header header

ScoredViewpoint[] viewpoints

string scoring_method      # e.g. "PLACEHOLDER_constant", "PLACEHOLDER_inverse_distance"
bool is_placeholder        # TRUE while the real Fisher IG formulation is TBD

# Pose the candidates were generated around (usually current OpenVINS pose)
geometry_msgs/Pose reference_pose

# Trace of the joint covariance the scores were derived from, purely so
# downstream logs can confirm the estimator was actually consuming covariance
# rather than silently running open-loop. -1 if no covariance received yet.
float64 covariance_trace
