# each Fusion object data 2225

uint16                  ID
uint16                  object_age
uint64                  time_stamp
uint16                  object_hidden_age
uint8                   classification
uint8                   classification_certainty
uint32                  calssifciation_age
float2D                 bounding_box_center    #cm
float2D                 bounding_box_size    #cm
float2D                 object_box_center    #cm
float2D                 object_box_center_sigma    #cm
float2D                 object_box_size    #cm
float32                 yaw_angle
float2D                 relative_velocity
float2D                 relative_velocity_sigma
float2D                 absolute_velocity  #cm/s
float2D                 absolute_velocity_sigma
uint8                   number_of_contour_points
uint8                   closest_point_index
float2D[]               list_of_contour_points  #cm
