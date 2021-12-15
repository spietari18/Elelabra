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

# GLOBALS
MYPATH=`script_path`
SRCDIR="$MYPATH/src"
TMPDIR="$MYPATH/build"
PRGOUT="$TMPDIR/program.hex"
DEVICE="atmega328p"

COMMON="-mmcu=${DEVICE} -Os -mcall-prologues -ffunction-sections -fdata-sections"
CFLAGS="${COMMON} -std=gnu99 -Wall -Wextra"
LDFLAGS="${COMMON} -Wl,-flto -mendup-at=main -lm"

action_compile()
{
	# make sure the build directory exists
	[ ! -d "$TMPDIR" ] && mkdir -p "$TMPDIR"

	# compile code
	objs=`find "$SRCDIR" -name '*.c' \
		| while read file; do
		sum="$(md5sum "$file" | cut -d\  -f1)"

		name="$(basename "$file")"

		src="${TMPDIR}/${sum}$(echo "$name" | grep -o '\.[^.]\+$')"
		obj="$(echo "$src" | sed -e 's/\.[^.]\+$/.o/g')"
		flg="${TMPDIR}/${sum}.cflags"

		echo "$obj"

		# recompile updated files
		if [ -f "$src" ] && \
			cmp -s "$src" "$file" && \
			echo "$CFLAGS" | cmp -s "$flg"; then
			echo "Using existing object for '$name'." 1>&2
			continue
		fi

		# compile
		echo "Compiling '$name'" 1>&2
		avr-gcc $CFLAGS -c -o "$obj" "$file" 2>&1 | sed -e 's/^/  /g' 1>&2 || exit 1
	
		# save parameters
		cp "$file" "$src"
		echo "$CFLAGS" > "$flg"
	done 3>&1`

	elf="`echo "$PRGOUT" | sed -e 's/\.[^.]\+$/.elf/g'`"

	# link objects
	echo "Linking objects:" 1>&2
	echo "$objs" | grep -o '[^/]\+$' | sed -e 's/^/  /g' 1>&2
	echo "$objs" | xargs -d '\n' avr-gcc $LDFLAGS -o "$elf" || exit 1

	# extract relevant sections from executable
	echo "Creating '`basename "$PRGOUT"`'" 1>&2
	avr-objcopy -j .text -j .data -O ihex "$elf" "$PRGOUT" || exit 1
	
	# get memory usage info with avr-size
	tmp=`avr-size -C --mcu="$DEVICE" "$elf"`
	[ "$?" -ne 0 ] && exit 1
	rom=`echo "$tmp" | grep 'Prog'`
	rom_abs=`echo "$rom" | grep -o '\s[0-9]\+' | tail -c +2`
	rom_per=`echo "$rom" | grep -o '[0-9.]\+%' | head -c -2`
	ram=`echo "$tmp" | grep 'Data'`
	ram_abs=`echo "$ram" | grep -o '\s[0-9]\+' | tail -c +2`
	ram_per=`echo "$ram" | grep -o '[0-9.]\+%' | head -c -2`

	echo "Memory usage:" 1>&2
	echo "  RAM: ${ram_abs} bytes (${ram_per}%)" 1>&2
	echo "  ROM: ${rom_abs} bytes (${rom_per}%)" 1>&2
}

# $1 = serial device
action_upload()
{
	# make sure program exists
	[ ! -f "$PRGOUT" ] && echo "Compile first." 2>&1 && exit 1

	# upload
	avrdude \
		-Dv \
		-p "$DEVICE" \
		-c arduino \
		-P "$1" \
		-b 115200 \
		-U flash:w:"$PRGOUT":i \
		-C 'avrdude.conf' \
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
