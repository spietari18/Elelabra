#!/bin/env python

# https://tabnabber.com/convert_guitar_sheet_music.asp

import sys

strings = ["e", "B", "G", "D", "A", "E"]

length = []
offset = []
lines  = []

for line in sys.stdin:
    _line = []

    for part in line.split(" "):
        part = part.strip()
        tabs = []
        i = 0

        for tab in part.split(","):
            if len(tab) in [2,3]:
                x = tab[0]
                y = tab[1:]

                if x in strings and y.isdigit():
                    tabs.append((x, int(y), i))
                    i += 1

        if len(tabs) < 1:
            continue

        _part = []

        for string in strings:
            for tmp in sorted( \
                filter(lambda x: x[0] == string, tabs), \
                    key=lambda x: x[2]):
                _part.append(tmp)

        _line.append(_part)

    if len(_line) > 0:
        lines.append(_line)

offset = []

for parts in lines:
    for i, part in enumerate(parts):
        if i >= len(offset):
            offset.append({})
            _off = offset[-1]
        else:
            _off = offset[i]

        for _, pos, off in sorted(part, key=lambda x: x[2]):
            tmp = len(str(pos))
            
            if off in _off:
                _off[off] = max(tmp, _off[off])
            else:
                _off[off] = tmp

for parts in lines:
    _lines = []

    for string in strings:
        _lines.append(string.lower() + "|")

    for i, part in enumerate(parts):
        _off = offset[i]

        for string, pos, off in sorted(part, key=lambda x: x[2]):
            j = strings.index(string)
            if j == -1:
                continue
            k = _off[off]
            
            i = 0
            while i < len(_lines):
                if i == j:
                    tmp = str(pos)
                    _lines[i] += tmp 
                    _lines[i] += "-"*(k - len(tmp))
                else:
                    _lines[i] += "-"*k
                _lines[i] += "-"
                i += 1


        i = 0
        while i < len(_lines):
            _lines[i] += "|"
            i += 1

    for line in _lines:
        print(line)
    print()
