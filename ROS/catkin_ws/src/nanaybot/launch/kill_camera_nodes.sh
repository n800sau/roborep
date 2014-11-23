#!/bin/sh

rosnode list|grep camera|xargs -n1 rosnode kill
