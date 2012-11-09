#!/bin/sh

gcc -I/usr/include/fastcgi -lfcgi server.c -o server
