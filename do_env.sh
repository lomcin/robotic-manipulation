#!/bin/bash

if [ -d "venv" ]; then
    source venv/bin/activate
else
    python3 -m pip install virtualenv
    virtualenv venv
    source venv/bin/activate
    python3 -m pip install --upgrade pip
    python3 -m pip install pygame scipy swig "gymnasium[all]" mujoco stable-baselines3
fi