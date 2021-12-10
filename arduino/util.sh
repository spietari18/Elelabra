#!/bin/sh

script_path()
{
        prg="$0"
        [ ! -f "$prg" ] && prg=`which -- "$prg"`
        echo "`cd \`dirname -- "$prg"\` && pwd`"
}

find_device()
{
	# enumerate serial devices
	sdevs=`find /dev -name 'ttyUSB*' -o -name 'ttyACM*'`
	count=`echo "$sdevs" | wc -w`

	# no serial devices
	if [ "$count" -lt 1 ]; then
		echo "No serial devices attached." 1>&2
		return 1;
	
	# multiple devices
	elif [ "$count" -gt 1 ]; then
		# we do this in a retarded way because
		# this way this script is compatible
		# with a standard POSIX shell
		echo "Found multiple serial devices:" 1>&2
		i=0
		echo "$sdevs" | while read 'device'; do
			echo " [${i}] ${device}" 1>&2
			i=`expr $i + 1` 
		done

		# read user input
		while read \
			-p "Select device [0-`expr $count - 1`]: " \
			'which'; do
			if [ \
				"$which" -ge 0 -a \
				"$which" -lt "$count" \
			] 2>/dev/null; then
				break
			else
				echo "Invalid input." 1>&2
			fi
		done

		# select device
		i=0
		echo "$sdevs" | while read 'device'; do
			if [ "$i" -eq "$which" ]; then
				echo "$device"
				break
			fi
			i=`expr $i + 1`
		done

	# one device
	else
		echo "$sdevs"
	fi
}


MY_PATH=`script_path`
HW="$MY_PATH/hw"
LIB="$MY_PATH/lib"
BUILD="$MY_PATH/build"
BOARD="arduino:avr:uno"

action_compile()
{
	# make sure the build dir exists
	[ ! -d "$BUILD" ] && mkdir -p "$BUILD"

	# build
	arduino-builder \
		-tools . \
		-fqbn "$BOARD" \
		-hardware "$HW" \
		-libraries "$LIB" \
		-build-path "$BUILD" \
		-warnings 'all' \
		-verbose \
		-prefs 'build.extra_flags=-DTIMER0_PRESCALER=64 -DSINGLE_ENTRY_POINT' \
		-compile \
		src/main.c `find src/ -type f \( -name '*.cpp' -o -name '*.c' \) -a \( -! -name 'main.c' \)` \
		|| exit 1

		# this enables a proper vfprintf()
		#-prefs 'compiler.ldflags=-Wl,-u,vfprintf -lm -lprintf_flt' \
}

# $1 = serial device
action_upload()
{
	# upload
	avrdude \
		-Dv \
		-p atmega328p \
		-c arduino \
		-P "$1" \
		-b 115200 \
		-U flash:w:"$BUILD/main.c.hex":i \
		-C "$HW/arduino/avr/avrdude.conf" \
		|| exit 1
}

# $1 = serial device
action_monitor()
{
	# this whole thing is needed because
	# CTRL+C on cat terminates the shell
	# by itself
	echo "Serial monitor (cat "$1"), CTRL+C to exit."
	cat "$1" </dev/null &
	trap "kill -TERM $!" INT
	wait $pid
	trap - INT
}

# we treat each parameter as an action
while [ ! -z "$1" ]; do
	case "$1" in
		# compilation doesn't require a device
		compile) "action_$1";;

		# upload and monitor require a device
		upload|monitor)
			# find device if one isn't selected 
			if [ -z "$device" ]; then
				device=`find_device`
				[ "$?" -ne 0 ] && exit 1
			fi
			"action_$1" "$device";;
		
		# allow changing device mid execution
		# NOP if only one device exists
		change_dev)
			if [ -z "$device" ]; then
				device=`find_device`
				[ "$?" -ne 0 ] && exit 1
			fi;;

		# print usage and exit
		*)
			echo "Unknown action '$1'." 1>&2
			echo "Usage: $0 [ACTION...]" 1>&2
			echo "ACTION can be one of: " 1>&2
			echo "compile, upload, monitor, change_dev" 1>&2
			exit 1;;
	esac
	shift 1
done
