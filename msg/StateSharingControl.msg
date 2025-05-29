uint64 timestamp

# Command enum
uint8 COMMAND_NONE = 0
uint8 COMMAND_START = 1
uint8 COMMAND_STOP = 2
uint8 COMMAND_UPDATE_PARAMS = 3

uint8 command

# Optional arguments (e.g., key-value pairs: "DELAY=1.0;PERIOD=0.5")
char[128] args
