std_msgs/Header header

string model_name
float64 inference_time_ms

# Keypoint coordinates — normalized [0.0, 1.0] relative to image size
float64[] kp_x
float64[] kp_y
float64[] kp_conf

int32 num_keypoints
bool is_detected

# Bounding box of all detected keypoints in pixels
float64 bbox_x1
float64 bbox_y1
float64 bbox_x2
float64 bbox_y2
float64 bbox_width_px
float64 bbox_height_px
