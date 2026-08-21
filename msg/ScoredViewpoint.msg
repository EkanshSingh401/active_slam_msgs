# One candidate viewpoint with its score breakdown.
#
# The individual terms are kept separate rather than pre-summed so the
# selector can re-weight them without the estimator re-scoring. How they get
# combined is an OPEN DESIGN DECISION -- see ScoredViewpointArray.

geometry_msgs/Pose pose

float64 score              # combined score, whatever the estimator chose
float64 information_gain   # expected Fisher information gain (PLACEHOLDER today)
float64 coverage_gain      # expected newly-observed volume (PLACEHOLDER today)
float64 travel_cost        # cost to reach from current pose
bool is_reachable          # false if ESDF says occupied / too close to obstacle
float64 clearance          # metres to nearest obstacle per ESDF, -1 if unknown
