gps_port="None"

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev)
do
	syspath="${sysdevpath%/dev}"
	devname="$(udevadm info -q name -p $syspath)"
	[[ "$devname" == "bus/"* ]] && continue
	eval "$(udevadm info -q property --export -p $syspath)"
	[[ -z "$ID_SERIAL" ]] && continue

	#id_serial below has to change accordingly
	if [[ $ID_SERIAL == *"Prolific_Technology_Inc._USB-Serial_Controller"* ]]
	then
		export gps_port="/dev/$devname"
		continue
	fi
done

roslaunch gps_common gps_navsat2odom.launch gps_dev:=$gps_port
