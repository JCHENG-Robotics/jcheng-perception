# Edge pipeline health. Counters are cumulative since node start unless the
# field name says otherwise.

std_msgs/Header header
string[] camera_ids
float32[] camera_fps
uint64 total_frames
uint64 total_detections
uint64 detections_matched
uint64 detections_dropped
float32 match_success_rate
float32 mean_detection_latency_ms
float32 mean_depth_match_age_ms
uint8 PHASE_SEARCH=0
uint8 PHASE_COARSE=1
uint8 PHASE_ACQUIRE=2
uint8 PHASE_FINE=3
uint8 PHASE_DROP_RECHECK=4
uint8 phase
bool gpu_alive
uint8 active_tracks
uint8 confirmed_tracks
string[] notes
