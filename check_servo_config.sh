#!/bin/bash

echo "=== Checking Hnuter Servo Configuration ==="
echo ""

echo "1. Checking airframe file parameters:"
grep -n "CA_SV_TL" ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter | head -20
echo ""

echo "2. Checking Gazebo servo function mapping:"
grep -n "SIM_GZ_SV_FUNC" ROMFS/px4fmu_common/init.d-posix/airframes/4051_gz_hnuter
echo ""

echo "3. Checking if custom controller is compiled:"
if [ -f "build/px4_sitl_default/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHnuter.cpp.o" ]; then
    echo "✓ ActuatorEffectivenessHnuter.cpp is compiled"
else
    echo "✗ ActuatorEffectivenessHnuter.cpp NOT compiled"
fi
echo ""

echo "4. Checking if Hnuter controller is registered:"
grep -n "HNUTER_TILTROTOR\|ActuatorEffectivenessHnuter" src/modules/control_allocator/ControlAllocator.cpp | head -5
echo ""

echo "5. To test in PX4 console, run these commands:"
echo "   param show CA_SV_TL_COUNT"
echo "   param show CA_AIRFRAME"
echo "   param show SYS_CTRL_ALLOC"
echo "   param show SIM_GZ_SV_FUNC1"
echo ""

echo "6. To monitor Gazebo topics:"
echo "   gz topic -l | grep servo"
echo "   gz topic -e -t /model/hnuter_0/servo_0"
echo ""

echo "=== End of Check ==="
