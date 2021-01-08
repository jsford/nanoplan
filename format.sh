#!/usr/bin/env bash

# Recurse over source files and apply clang-format to *.h *.hpp *.c *.cpp files.
find . -type d \( -path ./build -o -path ./include/nanoplan/ext \) -prune -false \
    -o -iname *.h -o -iname *.c -o -iname *.cpp -o -iname *.hpp \
    | xargs clang-format -style=file -i -fallback-style=none

exit 0

