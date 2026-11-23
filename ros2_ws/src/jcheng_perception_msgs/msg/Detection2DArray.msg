# Detections for one captured frame.
#
# header.stamp is the Pi 5 acquisition timestamp copied from the uplink
# CompressedImage. It is never rewritten with the inference time; the edge node
# uses it to retrieve the matching depth image and TF sample.

std_msgs/Header header
string camera_id
string model_id
uint64 frame_seq
uint32 image_width
uint32 image_height
float32 inference_ms
Detection2D[] detections
