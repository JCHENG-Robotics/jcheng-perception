# One 3-D target measurement from one camera at one capture time.

std_msgs/Header header
string camera_id
uint8 ROLE_UNKNOWN=0
uint8 ROLE_EXTERNAL=1
uint8 ROLE_WRIST=2
uint8 role
string label
uint32 class_id
float32 detection_score
float32 keypoint_score
# Pose in the target frame (nominally base_link). The covariance is 6x6
# row-major; roll/pitch are unobserved for a tabletop target.
geometry_msgs/PoseWithCovariance pose
float32 depth_m
float32 valid_depth_fraction
float32 occlusion_ratio
# Dynamic weight assigned during the two-view fusion (0..1).
float32 view_weight
uint64 frame_seq
# Camera optical frame that the pixels came from.
string camera_frame
