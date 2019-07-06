# Message containing a path and the desired gear for following the path.
nav_msgs/Path path # the path to follow
int8 gear # the gear to drive in, -1 = reverse, 0 = neutral, 1 = forward
