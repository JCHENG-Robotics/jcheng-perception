# GPU node health, one entry per configured camera.

std_msgs/Header header
string[] camera_ids
float32[] fps
float32[] inference_ms
uint64[] frames_received
uint64[] frames_dropped
uint64[] frames_failed
string[] model_ids
