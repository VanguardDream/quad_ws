# this msg is Abstraction "Leg" Layer. this msg shows Legs control reference. if you want see motor's detail, see MotorFeed.
Header DRIVE_FEED_HEAD

uint8 MODE
uint8 TROUGHS

float64 CONTACT_POSITION
float64 VELOCITY
float64 AMPLITUDE
float64 DUTY

MotorFeed[8] MOTORS
