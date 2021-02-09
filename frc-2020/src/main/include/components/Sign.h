#pragma once

namespace Sign {
	template <typename T> int Sign(T val) {
		return (T(0) < val) - (val < T(0));
	}
}
