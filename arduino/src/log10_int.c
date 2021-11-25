static uint32_t log10_table[] = {
/* (log10(N) - 1) << 16 - N (sizeof(s) = strlen(s) + 1) */
#define X(N) (((sizeof(#N) - 2) << 16) - N)
	X(0), X(0), X(0), X(10), X(10),
	X(10), X(100), X(100), X(100),
	X(1000), X(1000), X(1000), X(1000),
	X(10000), X(10000), X(10000)
#undef X
};

inline uint16_t log10_int(uint16_t n)
{
	return (((uint32_t)n + log10_table[8*sizeof(int) 
		- 1 - __builtin_clz(n)]) >> 16);
}
