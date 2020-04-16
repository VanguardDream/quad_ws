#Motor side feedback message

#Status feedback
#uint8 ID                #Motor ID set up.
# int16 current           #Current
# int32 position          #present posision
uint8 MODE
uint8 TROUGHS

float64 POSITION
float64 VELOCITY
float64 AMPLITUDE
float64 DUTY


#Error Code
bool isFault            #Motor is in fault?
uint8 error_code        #Error code
