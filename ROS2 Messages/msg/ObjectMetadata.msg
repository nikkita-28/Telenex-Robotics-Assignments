string object_type         # Type of object: "cylinder", "box", "sphere"
geometry_msgs/Point position  # X, Y, Z coordinates of the object
geometry_msgs/Quaternion orientation  # Orientation of the object
float32 size_x             # Size along X-axis
float32 size_y             # Size along Y-axis
float32 size_z             # Size along Z-axis
float32 friction           # Friction coefficient of the object
string material            # Material type (e.g., "metal", "plastic", "wood")
float32 weight             # Weight of the object (in kg)
bool accessible            # True if the object is accessible for grasping
sensor_msgs/Image camera_image  # Simulated camera image data 
