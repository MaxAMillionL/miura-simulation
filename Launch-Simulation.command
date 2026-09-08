#!/bin/zsh
cd -- "${0:A:h}" || exit 1
exec .venv/bin/mjpython -m miura_robot.run --viewer --paused --seconds 3600 --log output/inspection.csv
