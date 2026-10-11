#!/usr/bin/env python3
# Columbia - extracts the symbol table from a yaYUL assembly log.
# Output: one line per symbol: NAME TYPE BANK OFFSET
#   TYPE E = erasable (BANK 0-7, OFFSET 0-255), F = fixed (BANK 0-43,
#   OFFSET 0-1023). Usage: extract_symbols.py assemble.log > rope.sym
import re, sys

pat = re.compile(r'\d+,([EF]):\s+(\S+)\s+(?:(E?)(\d+),(\d+)|(\d+))\s')
out = {}
for line in open(sys.argv[1], errors='replace'):
    for m in pat.finditer(line + ' '):
        kind, name = m.group(1), m.group(2)
        if m.group(6) is not None:
            addr = int(m.group(6), 8)
            if kind == 'E':
                if addr >= 0o2000:
                    continue
                bank, off = addr >> 8, addr & 0o377
            else:
                if addr < 0o4000:
                    continue
                bank = 2 if addr < 0o6000 else 3
                off = addr & 0o1777
        else:
            bank, addr = int(m.group(4), 8), int(m.group(5), 8)
            if kind == 'E':
                off = addr & 0o377
            else:
                off = addr & 0o1777
        out[name] = (kind, bank, off)
for name in sorted(out):
    k, b, o = out[name]
    print('%s %s %d %d' % (name, k, b, o))
