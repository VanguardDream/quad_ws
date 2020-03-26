#This message_set is used to make quadnake drive.

#Driving and Gait mode
uint8 MODE=0            #Quadnake's motion modes
int8 FORWARD_DRIVE=0    #Percentage of driving speed for forward direction
int8 SIDE_DRIVE=0       #Percentage of driving speed for side direction
int8 ANGULAR_DRIVE=0    #Percentage of driving speed for angular direction

#Robot platform pose
int8 ROLL=0             #Platform adjusting X-axis
int8 YAW=0              #Platform adjusting Y-axis
int8 PITCH=0            #Platform adjusting Z-axis