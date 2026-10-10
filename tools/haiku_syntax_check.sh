#!/bin/sh
# Columbia — type-check the Haiku UI sources against Haiku's headers using
# the host compiler (no Haiku toolchain needed). Catches API misuse early;
# linking still needs Haiku.
#
# Usage: tools/haiku_syntax_check.sh /path/to/haiku/headers
set -e
H=${1:?usage: $0 /path/to/haiku/headers}
GV=$(g++ -dumpversion)
shim=$(mktemp -d)
cat > "$shim/shim.h" <<'EOS'
#include <locale.h>
typedef locale_t __locale_t;
#ifdef __cplusplus
extern "C" { int at_quick_exit(void (*)(void));
	void quick_exit(int) __attribute__((noreturn)); }
#endif
EOS
status=0
for f in app/*.cpp; do
	out=$(g++ -fsyntax-only -std=c++17 -Wall -Wextra -Wno-multichar \
		-Wno-deprecated-copy -D__HAIKU__ -D'__GLIBC_PREREQ(a,b)=0' -nostdinc \
		-I"$H/config" -I"$H" -I"$H/os" -I"$H/os/app" -I"$H/os/interface" \
		-I"$H/os/kernel" -I"$H/os/storage" -I"$H/os/support" -I"$H/os/locale" \
		-I"$H/os/translation" -I"$H/os/add-ons/graphics" -I"$H/os/drivers" \
		-Icore -Iapp \
		-isystem /usr/include/c++/$GV -isystem /usr/include/x86_64-linux-gnu/c++/$GV \
		-isystem "$H/posix" -isystem "$H/glibc" \
		-isystem /usr/lib/gcc/x86_64-linux-gnu/$GV/include \
		-include "$shim/shim.h" "$f" 2>&1 | grep -E '^app/' || true)
	if [ -n "$out" ]; then
		echo "$out"; status=1
	else
		echo "ok  $f"
	fi
done
rm -rf "$shim"
exit $status
