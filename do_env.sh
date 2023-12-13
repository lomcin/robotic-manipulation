#!/bin/bash

if [ -d "venv" ]; then
    source venv/bin/activate
else
    python3 -m pip install virtualenv
    virtualenv venv
    source venv/bin/activate
    python3 -m pip install --upgrade pip
    # Install torch with cuda
    python3 -m pip install torch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 --index-url https://download.pytorch.org/whl/cu121
    python3 -m pip install pygame scipy swig "gymnasium[all]" mujoco stable-baselines3[extra]
fi