#!/bin/bash

# usage: backtrace.sh 0x4008c470:0x3ffe3b00 0x4008c6a1:0x3ffe3b20 0x400e0244:0x3ffe3b40 0x400d54eb:0x3ffe3b70 0x400d33bc:0x3ffe3b90 0x400d33a5:0x3ffe3bb0 0x400ef87f:0x3ffe3bd0 0x40082983:0x3ffe3bf0 0x40082b78:0x3ffe3c20 0x40078f93:0x3ffe3c40 0x40078ff9:0x3ffe3c70 0x40079004:0x3ffe3ca0 0x400791a3:0x3ffe3cc0 0x400806ca:0x3ffe3df0 0x40007c31:0x3ffe3eb0 0x4000073d:0x3ffe3f20
# outputs a list of gdb commands to use to get meaningful backtrace.

while [ "x$1" != "x" ] ; do
    y=${1%%:*}
    echo x/i $y
    shift
done
