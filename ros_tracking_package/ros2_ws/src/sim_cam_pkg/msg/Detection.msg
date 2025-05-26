# Ref: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#

int32 og_image_width
int32 og_image_height
int32 class_id
string class_name
float32 score

# COCO Format: x, y, width, height
int32 box_x
int32 box_y
int32 box_width
int32 box_height
