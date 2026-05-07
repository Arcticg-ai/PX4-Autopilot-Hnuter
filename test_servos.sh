#!/bin/bash

echo "=== Testing Hnuter Servos ==="
echo ""
echo "This script will test each servo individually"
echo "Press Ctrl+C to stop"
echo ""

# Test servo 0 (rj2 - right arm primary tilt)
echo "Testing Servo 0 (rj2)..."
gz topic -t /model/hnuter_0/servo_0 -m gz.msgs.Double -p "data: 0.5" &
sleep 2
gz topic -t /model/hnuter_0/servo_0 -m gz.msgs.Double -p "data: 0.0" &
sleep 2

# Test servo 1 (lj2 - left arm primary tilt)
echo "Testing Servo 1 (lj2)..."
gz topic -t /model/hnuter_0/servo_1 -m gz.msgs.Double -p "data: 0.5" &
sleep 2
gz topic -t /model/hnuter_0/servo_1 -m gz.msgs.Double -p "data: 0.0" &
sleep 2

# Test servo 2 (rj1 - right arm secondary tilt)
echo "Testing Servo 2 (rj1)..."
gz topic -t /model/hnuter_0/servo_2 -m gz.msgs.Double -p "data: 0.5" &
sleep 2
gz topic -t /model/hnuter_0/servo_2 -m gz.msgs.Double -p "data: 0.0" &
sleep 2

# Test servo 3 (lj1 - left arm secondary tilt)
echo "Testing Servo 3 (lj1)..."
gz topic -t /model/hnuter_0/servo_3 -m gz.msgs.Double -p "data: 0.5" &
sleep 2
gz topic -t /model/hnuter_0/servo_3 -m gz.msgs.Double -p "data: 0.0" &
sleep 2

echo ""
echo "Test complete. Did you see the servos move?"
