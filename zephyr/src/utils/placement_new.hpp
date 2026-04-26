// zephyr/src/utils/placement_new.hpp
// Placement new — Zephyr's minimal newlib omits these from <new>.
// Include in any .cpp that uses placement-new syntax.
#pragma once

#include <stddef.h>

inline void* operator new(size_t, void* p) noexcept { return p; }
inline void  operator delete(void*, void*) noexcept {}
