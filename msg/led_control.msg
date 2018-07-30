# LED control: control a single or multiple LED's.
# These are the externally visible LED's, not the board LED's

# colors
uint8 COLOR_OFF = 0 # this is only used in the drivers
uint8 COLOR_RED = 1
uint8 COLOR_GREEN = 2
uint8 COLOR_BLUE = 3
uint8 COLOR_YELLOW = 4
uint8 COLOR_PURPLE = 5
uint8 COLOR_AMBER = 6
uint8 COLOR_CYAN = 7
uint8 COLOR_WHITE = 8

uint8 RST_COLOR_1 = 9     # r:255 g:0 b:0
uint8 RST_COLOR_2 = 10    # r:255 g:51 b:51
uint8 RST_COLOR_3 = 11    # r:255 g:128 b:0
uint8 RST_COLOR_4 = 12    # r:255 g:128 b:128
uint8 RST_COLOR_5 = 13    # r:255 g:204 b:51
uint8 RST_COLOR_6 = 14    # r:255 g:204 b:102
uint8 RST_COLOR_7 = 15    # r:255 g:255 b:0
uint8 RST_COLOR_8 = 16    # r:204 g:255 b:0
uint8 RST_COLOR_9 = 17    # r:204 g:255 b:153
uint8 RST_COLOR_10 = 18   # r:0 g:128 b:0

uint8 RST_COLOR_11 = 19   # r:0 g:189 b:0
uint8 RST_COLOR_12 = 20   # r:51 g:204 b:102
uint8 RST_COLOR_13 = 21   # r:0 g:255 b:0
uint8 RST_COLOR_14 = 22   # r:0 g:255 b:255
uint8 RST_COLOR_15 = 23   # r:64 g:224 b:208
uint8 RST_COLOR_16 = 24   # r:0 g:0 b:255
uint8 RST_COLOR_17 = 25   # r:51 g:153 b:204
uint8 RST_COLOR_18 = 26   # r:0 g:153 b:255
uint8 RST_COLOR_19 = 27   # r:102 g:153 b:255
uint8 RST_COLOR_20 = 28   # r:102 g:102 b:255

uint8 RST_COLOR_21 = 29   # r:51 g:102 b:255
uint8 RST_COLOR_22 = 30   # r:128 g:0 b:128
uint8 RST_COLOR_23 = 31   # r:153 g:0 b:255
uint8 RST_COLOR_24 = 32   # r:102 g:0 b:255
uint8 RST_COLOR_25 = 33   # r:255 g:0 b:255
uint8 RST_COLOR_26 = 34   # r:204 g:102 b:255
uint8 RST_COLOR_27 = 35   # r:255 g:128 b:192
uint8 RST_COLOR_28 = 36   # r:153 g:153 b:102
uint8 RST_COLOR_29 = 37   # r:64 g:128 b:128
uint8 RST_COLOR_30 = 38   # r:256 g:108 b:108

uint8 RST_COLOR_31 = 39   # r:153 g:102 b:0
uint8 RST_COLOR_32 = 40   # r:255 g:255 b:255

# LED modes definitions
uint8 MODE_OFF = 0 # turn LED off
uint8 MODE_ON = 1  # turn LED on
uint8 MODE_DISABLED = 2  # disable this priority (switch to lower priority setting)
uint8 MODE_BLINK_SLOW = 3
uint8 MODE_BLINK_NORMAL = 4
uint8 MODE_BLINK_FAST = 5
uint8 MODE_BREATHE = 6 # continuously increase & decrease brightness (solid color if driver does not support it)
uint8 MODE_FLASH = 7 # two fast blinks (on/off) with timing as in MODE_BLINK_FAST and then off for a while

uint8 MAX_PRIORITY = 2 # maxium priority (minimum is 0)


uint8 led_mask # bitmask which LED(s) to control, set to 0xff for all
uint8 color # see COLOR_*
uint8 mode # see MODE_*
uint8 num_blinks # how many times to blink (number of on-off cycles if mode is one of MODE_BLINK_*) . Set to 0 for infinite
                 # in MODE_FLASH it is the number of cycles (infinite is not possible)
uint8 priority # priority: higher priority events will override current lower priority events (see MAX_PRIORITY)
