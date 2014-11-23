#!/bin/sh

rosnode list|xargs -n1 rosnode kill
