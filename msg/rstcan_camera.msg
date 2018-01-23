uint8 RST_CAMERA_PTZ_CTRL = 1      #use "struct camera_ptz_ctrl" as data payload of "strcut rstcan_camera_s"
uint8 RST_CAMERA_SNAPSHOT = 2
uint8 RST_CAMERA_ZOOM_IN = 3
uint8 RST_CAMERA_ZOOM_OUT = 4
uint8 RST_CAMERA_ZOOM_STOP= 5
uint8 RST_CAMERA_RECORD_START = 6
uint8 RST_CAMERA_RECORD_STOP = 7


uint8 cmd           # camera actions
uint8[16] data
