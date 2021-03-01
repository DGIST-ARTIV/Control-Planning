Header header

uint32 id
std_msgs/String name # name : one of the V , L, VL    
std_msgs/Int32MultiArray bbox2d # bounding box [x,y,x,y]
std_msgs/Float32MultiArray bev # bird eye view , [x,y,x,y,x,y,x,y] (lidar) , [x,y,x,y] ( vision )
visualization_msgs/Marker bbox # bounding box 
geometry_msgs/Point point # object's closest pt (m)
uint32 inpath # inpath(1) not inpath(0)
std_msgs/Float32 distance # object's closest distance sqrt(x^2+y^2) ;
geometry_msgs/Point velocity #x,y,z velocity (m/s)
uint32 state # stationary(0) or moving(1) or don't know(2)
uint32 type_id # what is type(class)? with the document 
float32 ttc # time to collison

