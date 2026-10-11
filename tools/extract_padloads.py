#!/usr/bin/env python3
# Columbia — Apollo flight simulator for Haiku
# Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
#
# Extracts the pad-loaded erasables of an AGC program from a saved erasable
# memory image and writes them as a C++ table that gnc::PadLoader applies by
# name.
#
# Which erasables are pad loads comes from the program's own
# ERASABLE_ASSIGNMENTS listing (Virtual AGC transcription): every name in a
# section headed "... PAD LOADED ..." / "... PADLOADS ...", and every name
# whose comment carries the "PL" (pad loaded) tag. Addresses come from the
# symbol table Columbia extracts from the yaYUL assembly (rope/sym/*.sym).
#
# The erasable images are mission data: the values loaded into the CMC and
# LGC before flight, as reconstructed from NASA mission documentation. The
# NASSP project's mission scenarios (Orbiter "Project Apollo") carry such
# images as EMEMnnnn lines, which is the format read here.
#
# Usage:
#   extract_padloads.py --erasable ERASABLE_ASSIGNMENTS.agc --sym X.sym \
#       --scenario file.scn --ship Eagle: --name Apollo11_LGC [--check N=V ...]

import argparse
import re
import sys

SKIP = {
    # Dynamic state Columbia computes for itself at mission setup.
    'RRECTCSM', 'VRECTCSM', 'RCVCSM', 'VCVCSM', 'TETCSM', 'TCCSM', 'XKEPCSM',
    'RRECTLEM', 'VRECTLEM', 'RCVLEM', 'VCVLEM', 'TETLEM', 'TCLEM', 'XKEPLEM',
    'DELTACSM', 'DELTALEM', 'NUVCSM', 'NUVLEM', 'REFSMMAT', 'TEPHEM',
    'TLAND', 'RLS', 'MASS', 'LEMMASS', 'CSMMASS', 'AZO', '-AYO', 'AXO',
    '504LM', 'FLAGWRD3', 'FLAGWRD8', 'FLGWRD10',
    # Temporaries that share a pad-load section's storage.
    'ABVEL*', 'VSELECT*',
}


def pad_names(path):
    names = []
    in_pad = False
    for raw in open(path, errors='replace'):
        line = raw.rstrip('\n')
        if line.startswith('#'):
            if re.search(r'\(\d+D\)', line):
                up = line.upper()
                in_pad = ('PAD' in up and 'OVERLAY' not in up)
            continue
        m = re.match(r'^([^\s#]+)\s+(ERASE|EQUALS|=)\s*([^#]*)(#.*)?$', line)
        if not m:
            continue
        name, op, operand, comment = m.group(1), m.group(2), m.group(3), m.group(4) or ''
        tagged = re.search(r'\bPL\b|\)PL\b|PL,', comment) is not None
        if not (in_pad or tagged):
            continue
        size = 1
        if op == 'ERASE':
            mm = re.match(r'\s*\+\s*(\d+)D?', operand)
            if mm:
                size = int(mm.group(1)) + 1
        else:
            mm = re.search(r'[IB]?\((\d+)\)', comment)
            if mm:
                size = int(mm.group(1))
        if size > 40:      # whole regions (W matrix and the like)
            continue
        names.append((name, size))
    return names


def load_image(path, ship):
    lines = open(path, errors='replace').read().split('\n')
    start = None
    for i, l in enumerate(lines):
        if l.startswith(ship):
            start = i
    if start is None:
        sys.exit('ship %s not found in %s' % (ship, path))
    mem = {}
    for l in lines[start + 1:]:
        if re.match(r'^\S+:ProjectApollo', l):
            break
        m = re.match(r'\s*EMEM([0-7]+)\s+([0-7]+)', l)
        if m:
            mem[int(m.group(1), 8)] = int(m.group(2), 8)
    return mem


def load_syms(path):
    syms = {}
    for l in open(path):
        p = l.split()
        if len(p) == 4 and p[1] == 'E':
            syms[p[0]] = int(p[2]) * 256 + int(p[3])
    return syms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--erasable', required=True)
    ap.add_argument('--sym', required=True)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--ship', required=True)
    ap.add_argument('--name', required=True)
    ap.add_argument('--check', action='append', default=[],
        help='NAME=OCTAL sanity check of the image/symbol pairing')
    a = ap.parse_args()
    syms = load_syms(a.sym)
    mem = load_image(a.scenario, a.ship)
    for c in a.check:
        n, v = c.split('=')
        got = mem.get(syms.get(n, -1))
        if got != int(v, 8):
            sys.exit('check failed: %s = %s, expected %s' % (n,
                'missing' if got is None else '%05o' % got, v))
    out = []
    seen = set()
    for name, size in pad_names(a.erasable):
        if (name in SKIP or name in seen or name not in syms
                or name.startswith('END-')):
            continue
        seen.add(name)
        for i in range(size):
            # Images omit zero words.
            out.append((name, i, mem.get(syms[name] + i, 0)))
    print('// %s: %d pad-loaded words' % (a.name, len(out)))
    print('const PadWord k%s[] = {' % a.name)
    for name, i, v in out:
        print('\t{"%s", %d, 0%05o},' % (name.replace('\\', '\\\\'), i, v))
    print('\t{nullptr, 0, 0}')
    print('};')


if __name__ == '__main__':
    main()
