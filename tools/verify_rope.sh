#!/bin/sh
# Columbia — rebuild and verify the rope images from the Virtual AGC sources.
#
# Assembles every program in rope/ with yaYUL and checks that:
#   1. each assembles with no errors,
#   2. each matches the independent octal transcription of the flight rope,
#      where Virtual AGC has one,
#   3. each matches the image shipped in rope/.
# It also builds the AGC self-test rope (Validation) used by `make test`.
#
# Virtual AGC's tools are GPL; they are only used here, at verification time,
# and nothing from them is linked into Columbia.
#
# Usage: tools/verify_rope.sh [work-dir]

set -e
here=$(cd "$(dirname "$0")/.." && pwd)
work=${1:-$here/objects/virtualagc}
PROGRAMS=$(cd "$here/rope" && ls *.bin | sed 's/\.bin$//')

if [ ! -d "$work/.git" ]; then
	git clone --depth 1 --filter=blob:none --sparse \
		https://github.com/virtualagc/virtualagc.git "$work"
	(cd "$work" && git sparse-checkout set $PROGRAMS \
		Validation yaYUL Tools)
fi
cd "$work"
echo "Virtual AGC revision: $(git log -1 --format='%H %ci')"

cc -O2 -w -DNVER='"columbia"' -o yaYUL/yaYUL yaYUL/*.c -lm
cc -O2 -w -DNVER='"columbia"' -o Tools/oct2bin Tools/oct2bin.c Tools/utils.c

status=0
for program in $PROGRAMS; do
	(cd $program && ../yaYUL/yaYUL MAIN.agc > assemble.log 2>&1) || true
	errors=$(grep -E '^Fatal errors:' $program/assemble.log | awk '{print $3}')
	if [ -f $program/$program.binsource ]; then
		(cd $program && ../Tools/oct2bin < $program.binsource > /dev/null \
			&& mv oct2bin.bin octal.bin)
		if cmp -s $program/MAIN.agc.bin $program/octal.bin; then
			octal="matches octal rope"
		else
			octal="DIFFERS from octal rope"; status=1
		fi
	else
		octal="no octal listing"
	fi
	if cmp -s $program/MAIN.agc.bin "$here/rope/$program.bin"; then
		shipped="matches rope/$program.bin"
	else
		shipped="DIFFERS from rope/$program.bin"; status=1
	fi
	echo "$program: $errors fatal errors; $octal; $shipped"
done

(cd Validation && ../yaYUL/yaYUL Validation.agc > assemble.log 2>&1)
echo "Self-test rope: $work/Validation/Validation.agc.bin"
echo "Run it with: make test VALIDATION=$work/Validation/Validation.agc.bin"
exit $status
