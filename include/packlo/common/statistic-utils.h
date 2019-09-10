#pragma once

#include <chrono>
#include <utility>
#include <functional>

namespace common {

template <typename F, typename... Args> 
double executeTimedFunction(F&& f, Args&&... args) {
  auto start = std::chrono::steady_clock::now();
	std::bind(std::forward<F>(f), std::forward<Args>(args)...)();
  auto end = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(
			end - start).count();
}

}
