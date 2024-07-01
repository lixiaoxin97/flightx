# Sbus messages

# Channel data constants
# [192, 992, 1792] corresponds to [1000, 1500, 2000] on FC
uint16 KNCHANNELS = 16
uint16 KMINCMD = 192
uint16 KMEANCMD = 992
uint16 KMAXCMD = 1792

# Channel mapping constants
uint8 KTHROTTLE = 0
uint8 KROLL = 1
uint8 KPITCH = 2
uint8 KYAW = 3
uint8 KCONNECTEDSTATE = 4
uint8 KBRIDGESTATE = 5
uint8 KARMSTATE = 6
uint8 KCONTROLMODE = 7

# Regular 16 sbus channels with 11 bit value range each
uint16[16] channels

# Digital channels (ch17 and ch18)
bool digital_channel_1
bool digital_channel_2

# Frame lost flag
bool frame_lost

# Failsafe flag
bool failsafe