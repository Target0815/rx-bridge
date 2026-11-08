#pragma once
#define UNUSED(x) (void)(x)
#define PACKED __attribute__((packed))
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif
#define STATIC_ASSERT(cond, msg) _Static_assert(cond, #msg)
#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define BV(x) (1 << (x))
