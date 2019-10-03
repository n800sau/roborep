# bottom
rosservice call /ot/twist_scan '{scan_id: 1, tilt: 140, scan_attempts: 10}' > twist_scan140.log
# top
rosservice call /ot/twist_scan '{scan_id: 1, tilt: 40, scan_attempts: 10}' > twist_scan040.log
# middle
rosservice call /ot/twist_scan '{scan_id: 1, tilt: 110, scan_attempts: 10}' > twist_scan110.log
#rosservice call /ot/twist_scan '{scan_id: 1, tilt: 120, scan_attempts: 10}' &> twist_scan.log
echo $?

