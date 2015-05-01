#!/bin/sh

redis-cli publish command '{"name": "turn", "params": {"rad": 270}}'
echo exit $?
