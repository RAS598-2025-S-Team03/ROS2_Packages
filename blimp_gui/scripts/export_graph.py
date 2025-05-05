#!/usr/bin/env python3
import subprocess
import time
import os

OUTPUT = "/home/nihar/blimp_ws/src/blimp_gui/static/ros_graph.png"

while True:
    try:
        print("Capturing rqt_graph...")
        subprocess.run([
            "xvfb-run", "-a", "rqt", "--standalone", "rqt_graph"
        ], timeout=8)
        subprocess.run([
            "import", "-window", "root", OUTPUT
        ])
    except subprocess.TimeoutExpired:
        print("Timeout reached. Proceeding to next refresh.")
    time.sleep(15)

