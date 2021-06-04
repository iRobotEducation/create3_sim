# This message describes a detected ir opcode.

uint8 CODE_IR_FORCE_FIELD=161
uint8 CODE_IR_VIRTUAL_WALL=162
uint8 CODE_IR_BUOY_GREEN=164
uint8 CODE_IR_BUOY_RED=168
uint8 CODE_IR_BUOY_BOTH=172
uint8 CODE_IR_EVAC_GREEN_FIELD=244
uint8 CODE_IR_EVAC_RED_FIELD=248
uint8 CODE_IR_EVAC_BOTH_FIELD=252

uint8 SENSOR_OMNI=0
uint8 SENSOR_DIRECTIONAL_FRONT=1

std_msgs/Header header      # time when we saw the opcode
uint8 opcode                # opcode
uint8 sensor                # the sensor which saw the opcode
