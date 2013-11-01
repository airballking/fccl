# name of the feature used for identification in knowledge base
std_msgs/String name

# name of tf-frame w.r.t. which the feature is defined
std_msgs/String reference

# type of the feature with predefined constants
uint8 UNKNOWN_FEATURE=0
uint8 POINT=1
uint8 LINE=2
uint8 PLANE=3

std_msgs/UInt8 type

# position of the feature
geometry_msgs/Vector3 position

# orientation of the feature
# - ignored for points
# - direction vector for lines
# - normal vector for planes
geometry_msgs/Vector3 direction
