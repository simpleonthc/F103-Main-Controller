#pragma once
#include <stdio.h>

#ifdef DEBUG
	#define dbg_printf(...) printf(__VA_ARGS__)
	#define dbg_break() __BKPT(0)
#else
	#define dbg_printf(...) (void(0))
	#define dbg_break() (void(0))
#endif
