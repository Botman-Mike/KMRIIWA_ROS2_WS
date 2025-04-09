#!/bin/bash
export DISPLAY=:1

echo "Testing X11 display at $DISPLAY"
echo "If a window appears, your X11 display is working correctly."

# Try to run a simple X11 application
if command -v xeyes &> /dev/null; then
    xeyes
elif command -v xclock &> /dev/null; then
    xclock
elif command -v xterm &> /dev/null; then
    xterm -e "echo X11 is working! Press Enter to close.; read"
else
    echo "No X11 test applications found. Please install x11-apps package."
    echo "sudo apt-get install x11-apps"
fi
