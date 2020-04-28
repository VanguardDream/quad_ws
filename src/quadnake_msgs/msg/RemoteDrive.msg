#This message_set is used to make quadnake drive.

#Driving and Gait mode
uint8 MODE            #Quadnake's motion modes
int8 FORWARD_DRIVE    #Percentage of driving speed for forward direction
int8 SIDE_DRIVE       #Percentage of driving speed for side direction
int8 ANGULAR_DRIVE    #Percentage of driving speed for angular direction

#Robot platform pose
int8 roll             #Platform adjusting X-axis
int8 yaw              #Platform adjusting Y-axis
int8 pitch            #Platform adjusting Z-axis