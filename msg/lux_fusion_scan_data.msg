lux_header   header

#Ibeo FUSION SYSTEM/ECU scan data 2204

uint64                  scan_start_time
uint32                  scan_end_time_offset
uint32                  flags
uint16                  scan_number
uint16                  num_scan_pts
uint8                   num_scan_info
lux_fusion_scan_info[]  scan_info_list
lux_fusion_scan_point[] scan_point_list
