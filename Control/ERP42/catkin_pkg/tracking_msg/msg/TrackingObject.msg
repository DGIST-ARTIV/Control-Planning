Header header

uint32 id
std_msgs/String name # name id
std_msgs/Int32MultiArray bbox2d # bounding box [x,y,x,y]
std_msgs/Float32MultiArray bev # bird eye view , [x,y,x,y,x,y,x,y]
visualization_msgs/Marker bbox # bounding box
geometry_msgs/Point point # object's closest pt (m)
geometry_msgs/Point velocity #x,y,z velocity (m/s)
uint32 state # stationary(0) or moving(1)
uint32 type_id # what is type? with the document
float32 ttc # time to collison
#uint32 inpath # not in my path(0) or in my path(1)
