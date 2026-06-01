std_msgs/Header header

string model_name
float64 inference_time
float64 confidence_score
bool is_detected
float64 distance_estimate

# Per-keypoint data (17 COCO keypoints)
# Indices: 0=nose 1=left_eye 2=right_eye 3=left_ear 4=right_ear
#          5=left_shoulder 6=right_shoulder 7=left_elbow 8=right_elbow
#          9=left_wrist 10=right_wrist 11=left_hip 12=right_hip
#          13=left_knee 14=right_knee 15=left_ankle 16=right_ankle
float64[] keypoint_scores
float64[] keypoint_x
float64[] keypoint_y

# Fraction of critical keypoints (shoulders 5-6, elbows 7-8, wrists 9-10)
# detected above threshold (0.0 - 1.0)
float64 skeleton_completeness

# Ground-truth robot-to-person distance set by the benchmark script (m)
float64 robot_distance_gt
