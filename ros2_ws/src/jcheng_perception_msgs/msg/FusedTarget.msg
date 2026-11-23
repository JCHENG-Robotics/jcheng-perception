# A fused target ready for the planner.

std_msgs/Header header
string target_id
string label
uint32 class_id
geometry_msgs/PoseWithCovariance pose
float32 confidence
uint8 PHASE_SEARCH=0
uint8 PHASE_COARSE=1
uint8 PHASE_ACQUIRE=2
uint8 PHASE_FINE=3
uint8 PHASE_DROP_RECHECK=4
uint8 phase
uint8 STATUS_TENTATIVE=0
uint8 STATUS_CONFIRMED=1
uint8 STATUS_COASTING=2
uint8 STATUS_LOST=3
uint8 status
float32 weight_external
float32 weight_wrist
float32 depth_external_m
float32 depth_wrist_m
bool occluded
bool recheck_required
string[] source_cameras
float32 age_s
