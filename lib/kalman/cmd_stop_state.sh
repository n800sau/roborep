#!/bin/sh

redis-cli lpush cmd.kalman '{"cmd": "stop_state"}'
