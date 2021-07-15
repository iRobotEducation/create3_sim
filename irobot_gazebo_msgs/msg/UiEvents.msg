# Event ID
uint8 event

# Type of Events
uint8 DUMMY_EVENT    = 0
uint8 HOME_BUTTON    = 1
uint8 MOW_BUTTON     = 2
uint8 KEYSWITCH_MODE = 3
uint8 ROBOT_LID      = 4
uint8 STOP_BUTTON    = 5

# States
uint16[] states

# Possible States for the Events
uint8 NONE   = 0
uint8 OFF    = 1
uint8 ON     = 2
uint8 AUTO   = 3
uint8 MANUAL = 4
uint8 OPEN   = 5
uint8 CLOSED = 6