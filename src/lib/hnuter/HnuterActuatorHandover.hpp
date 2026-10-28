/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/
#pragma once

#include <cmath>
#include <cstdint>

// No uORB dependency: the same ownership/freshness policy is exercised offline.
class HnuterActuatorHandover
{
public:
	void update(bool armed, bool direct, uint64_t timestamp)
	{
		if (!armed) {
			_direct = false;
			_pending = false;
			_entry = _exit = 0;
			return;
		}

		if (direct && !_direct) {
			_entry = timestamp;
			_pending = false;
		}

		if (!direct && _direct) {
			_exit = timestamp;
			_pending = true;
		}

		_direct = direct;
	}

	bool direct() const { return _direct; }
	bool pending() const { return _pending; }
	uint64_t exitTime() const { return _exit; }
	void complete() { _pending = false; }

	bool validSnapshot(uint64_t timestamp, uint64_t now, const float *values, unsigned count,
			   float minimum = -1.f) const
	{
		if (!_pending || _entry == 0 || timestamp <= _entry || timestamp > _exit
		    || timestamp > now || now - timestamp > 100000) {
			return false;
		}

		for (unsigned i = 0; i < count; ++i) {
			if (!std::isfinite(values[i]) || values[i] < minimum || values[i] > 1.f) {
				return false;
			}
		}

		return true;
	}

private:
	bool _direct{false};
	bool _pending{false};
	uint64_t _entry{0};
	uint64_t _exit{0};
};
