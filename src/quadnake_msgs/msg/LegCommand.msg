#This message_set is used to make quadnake leg control.

#Driving and Gait mode
uint8 MODE              #Quadnake's leg motion modes
uint8 TROUGHS           #Number of troughs

float64 POSTION         #Position of trough of the leg
float64 VELOCITY        #Velocity of trough
float64 AMPLITUDE       #Amplitude of the motion wave
float64 DUTY            #Duty cycle of the motion wave