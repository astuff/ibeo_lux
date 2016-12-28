# each object data 2221

uint16                  ID
uint16                  age
uint16                  prediction_age
uint16                  relative_timestamp
point2D                 reference_point    #cm
point2D                 reference_point_sigma    #cm
point2D                 closest_point    #cm
point2D                 bounding_box_center    #cm
uint16                  bounding_box_width    #cm
uint16                  bounding_box_length    #cm
point2D                 object_box_center    #cm
size2D                  object_box_size    #cm
int16                   object_box_orientation
point2D                 absolute_velocity  #cm/s
size2D                  absolute_velocity_sigma
point2D                 relative_velocity
#0:unclassified;1:unknown small;2:unknown big;3:pedestrain;4:bike;5:car;6:truck;7:reserved
uint16                  classification           
uint16                  classification_age
uint16                  classification_certaiinty
uint16                  number_of_contour_points
point2D[]               list_of_contour_points  #cm


