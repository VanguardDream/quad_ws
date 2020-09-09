#Motor side feedback message

#Status feedback
#uint8 ID                #Motor ID set up.
# int16 current           #Current
# int32 position          #present posision
uint8 MODE
uint8 TROUGHS

int16 CURRENT
int32 POSITION

float64 CONTACT_POSITION
float64 VELOCITY
float64 AMPLITUDE
float64 DUTY


#Error Code
bool isFault            #Motor is in fault?
uint8 error_code        #Error code
