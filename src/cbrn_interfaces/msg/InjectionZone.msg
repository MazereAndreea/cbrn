std_msgs/Header header

# Identified injection zone
string zone_type        # deltoid_left | deltoid_right | arm_mid_left | arm_mid_right | unknown
string body_position    # face_up | face_down | lateral_left | lateral_right | unknown
float64 pixel_x         # injection point in image (pixels)
float64 pixel_y
float64 confidence      # confidence of injection zone classification

# Distance estimation (from bbox + optional LiDAR)
float64 distance_m

# Nav2Goal in map frame — filled by injection_zone_analyzer
float64 nav_goal_x
float64 nav_goal_y
float64 nav_goal_yaw    # approach yaw (rad)
bool goal_valid
