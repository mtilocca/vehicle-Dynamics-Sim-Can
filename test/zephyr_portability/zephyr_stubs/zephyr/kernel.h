#pragma once
// Minimal Zephyr kernel.h stub for host-side unit tests.
#include <stdint.h>
typedef long atomic_val_t;
typedef struct { atomic_val_t val; } atomic_t;
#define ATOMIC_INIT(x) { (x) }
static inline atomic_val_t atomic_get(const atomic_t* a) { return a->val; }
