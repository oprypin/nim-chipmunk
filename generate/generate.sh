#!/bin/sh

cpp -C chipmunk/chipmunk.h > 1.h &&
cat 1.h | python3 preprocess.py > 2.h &&
c2nim 2.h --dynlib:lib --cdecl --out:3.nim &&
cat 3.nim | python3 postprocess.py > 4.nim &&
cp -r 4.nim ../chipmunk.nim
