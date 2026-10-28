#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build="$root/build/px4_sitl_default"
out="$build/hnuter_handover_test"
flags=(-std=gnu++14 -O0 -g -ffunction-sections -fdata-sections
       -D__PX4_POSIX -D__PX4_LINUX -DCONFIG_ARCH_BOARD_PX4_SITL -DENABLE_LOCKSTEP_SCHEDULER
       '-DMODULE_NAME="hnuter_handover_test"' -D__STDC_FORMAT_MACROS -include visibility.h)
for path in boards/px4/sitl/src platforms/posix/src/px4/common/include platforms/posix/src/px4/generic/generic/include platforms/common platforms/common/include src src/include src/lib src/lib/matrix src/modules platforms/posix/include; do
    flags+=("-I$root/$path")
done
flags+=("-I$build" "-I$build/src/lib")
if rg -q 'bool ground_contact' "$root/src/modules/mc_rate_control/HnuterControl.hpp"; then
    flags+=(-DHNUTER_HAS_GROUND_CONTACT)
fi
mapfile -d '' libs < <(find "$build" -name '*.a' -print0)
c++ "${flags[@]}" "$root/test/hnuter_handover/HnuterHandoverTest.cpp" \
    -Wl,--gc-sections -Wl,--wrap=hrt_absolute_time -Wl,--start-group "${libs[@]}" -Wl,--end-group \
    -lpthread -ldl -lrt -lm -o "$out"
"$out"
