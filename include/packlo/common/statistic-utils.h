#pragma once

#include <chrono>
#include <utility>
#include <iostream>
#include <functional>

namespace common {

template <typename F, typename... Args> 
double executeTimedFunction(F&& f, Args&&... args) {
	auto task = std::bind((std::forward<F>(f)), std::forward<Args>(args)...);
  auto start = std::chrono::high_resolution_clock::now();
	task();
  auto end = std::chrono::high_resolution_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(
			end - start).count();
}

}
