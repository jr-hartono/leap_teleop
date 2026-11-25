# Pose change since engagement, the input of every robot bridge. Translation is added to the
# latched tool position; rotation is composed in base axes about the tool origin.

std_msgs/Header header
bool engaged
geometry_msgs/Transform delta
