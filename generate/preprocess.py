# Copyright (C) 2015 Oleh Prypin <blaxpirit@gmail.com>
# Licensed under terms of MIT license (see LICENSE)

import sys
import re

src = []

outputting = False
for line in sys.stdin:
    line = line.strip('\n')
    sline = line.strip()
    m = re.match(r'^# [0-9]+ "(.+)"( [0-9])*$', line)
    if m:
        fn = m.group(1)
        outputting = fn.startswith('chipmunk/')
        continue
    if not outputting:
        continue
    if '@{' in line or '@}' in line:
        continue
    src.append(line)

src = '\n'.join(src)

# From GCC infinity to a generic name
src = re.sub(r'\b__builtin_inff\(\)', 'INFINITY', src)

# Make sure empty structs are still present
src = re.sub(r'typedef struct (cp\w+) \1;', r'struct \1 {};', src)

# Remove unneeded comments
src = re.sub(r'/\* Copyright.+?\*/', '', src, flags=re.MULTILINE|re.DOTALL)
src = re.sub(r'/// *@defgroup \w+ (.+)(\n///.+)*', r'', src)
src = re.sub(r'/\*\*\s*@defgroup \w+ ([^\n]+).+\*/', r'', src, flags=re.DOTALL)

# Remove non-doc comments
src = re.sub(r'^\s*//[^/].+', '', src, flags=re.MULTILINE)

# Remove dangling comments
#src = re.sub(r'\n\n(///.+\n)+\n', '\n\n', src, flags=re.MULTILINE)

# Fix comments
#src = re.sub(r'^((?:///.+\n)+)((?:.+\n)+)', r'\2\1', src, flags=re.MULTILINE)
#src = re.sub(r'^((?: ///.+\n)+)((?: .+\n))', r'\2\1', src, flags=re.MULTILINE)
#src = re.sub(r'}\n((?: *///.+\n)+)', r'\1}', src)

# Remove comments
#src = re.sub(r' *//.+', '', src, flags=re.MULTILINE)

print(src)