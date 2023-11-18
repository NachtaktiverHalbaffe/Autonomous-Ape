# The registered namespace of the robot
string name_space
a# The current position of the robot. It's internally provided by a corresponding subscription of the pose of the corresponding robot => should be precise
geometry_msgs/PoseWithCovarianceStamped pose
# The current planned global path of the robot. The robots sends them to the multi robot coordinator when planned, so path data could be a little bit old
nav_msgs/Path nav_path
# If the robot is currently navigating
bool is_navigating
# The current steering command
geometry_msgs/Twist velocity