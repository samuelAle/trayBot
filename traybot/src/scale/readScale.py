#!/usr/bin/env python

import time
import os

f = open("scaleData.txt")

while 1:
    where = f.tell()
    line = f.readline()
    if not line:
        time.sleep(1)
        f.seek(where)
    else:
        print line, # already has newline
"""
while 1:
    print os.system("cat /dev/hidraw3 | head -c 7");
"""
