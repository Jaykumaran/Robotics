# Ref:https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#

# Unique ID of the tracked object
int32 id

# Object class name from detection
string class_name

# Confidence score from the original detection
float32 score

# Current bounding box (from the latest matched detection)
int32 box_x
int32 box_y
int32 box_width
int32 box_height

# Current centroid of the object (Kalman Filter state)
float32 centroid_x
float32 centroid_y

# Trajectory: A list of historical centroids
geometry_msgs/Point[] trajectory_points

# Number of frames since this object was last directly associated with a detection
int32 frames_since_last_seen
