 int32 lat # Latitude,expressed as degree * 1E7
 int32 lon # Longitude,expressed as degree * 1E7
 int32 alt # < Altitude in meters, expressed as * 1E3(millimeters)
 int32 relative_alt # Altitude above ground in meters, expressed as * 1E3(millimeters)
 uint16 voltage_battery # Battery volatage, in millivolts(1 = 1 millivolt)
 uint16 param_index # Index of this onboard parameter
 uint8 fix_type # 0:None 1:GPS Lock 2:RTK float 3:RTK Fixed
 int8 battery_remaining # Remaining battery energy:(0%:0,100%:100),-1:qutopilot estimate the remaining battery
 uint8 param1 # Reserved
 uint8 param2 # Reserved
 uint8 param3 # Reserved
 uint8 param4 # Reserved