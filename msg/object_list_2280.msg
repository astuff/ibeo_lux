# each Fusion object data 2280

uint16                  ID
uint16                  flags
uint32                  object_age
uint64                  time_stamp
uint16                  object_prediction_age
uint8                   classification
uint8                   classification_quality
uint32                  calssifciation_age
float2D                 object_box_center    #cm
float2D                 object_box_size    #cm
float32                 object_course_angle
float32                 object_course_angle_sigma
float2D                 relative_velocity
float2D                 relative_velocity_sigma
float2D                 absolute_velocity  #cm/s
float2D                 absolute_velocity_sigma
uint8                   number_of_contour_points
uint8                   closest_point_index
uint16                  reference_point_location
float2D                 reference_point_coordinate
float2D                 reference_point_coordinate_sigma
uint16                  object_priority
float2D[]               list_of_contour_points  #cm
