#!/usr/bin/env python3

import os
import sys
import shutil
from pathlib import Path

# Get the paths
ws_path = Path('/home/ros2-control/KMRIIWA_ROS2_WS')
src_dir = ws_path / 'src' / 'kmr_communication' / 'nodes'
install_dir = ws_path / 'install' / 'kmr_communication' / 'lib' / 'kmr_communication'

# Create the install directory if it doesn't exist
os.makedirs(install_dir, exist_ok=True)

# Files to copy
files_to_copy = [
    'communication_health_monitor.py',
    'safety_monitor.py',
    'emergency_stop_bridge.py'
]

# Copy each file and make it executable
for file in files_to_copy:
    src_file = src_dir / file
    dest_file = install_dir / file
    
    if src_file.exists():
        print(f"Copying {src_file} to {dest_file}")
        shutil.copy2(src_file, dest_file)
        os.chmod(dest_file, 0o755)  # Make executable
        print(f"Made {dest_file} executable")
    else:
        print(f"Warning: Source file {src_file} does not exist, skipping.")

print("Setup complete!")
