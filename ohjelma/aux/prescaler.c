/* prescaler selection code testing */

#include <stdio.h>
#include <stdlib.h>

const struct {
	unsigned short fact;
	unsigned char  bits;
} prescalers[] = {
	{1024, 0b101},
	{ 256, 0b100},
	{  64, 0b011},
	{   8, 0b010},
	{   1, 0b001}
};

#define ARRAY_SIZE(A) \
	(sizeof(A)/sizeof(*(A)))

#define CLOCK_FREQ 16000000UL

void select_prescaler(unsigned int freq, unsigned short max)
{
	size_t i;

	unsigned int res, old, div;

	i = 0;
	old = max;
	while (i < ARRAY_SIZE(prescalers))
	{
		div = freq*prescalers[i].fact;
		res = (CLOCK_FREQ + (div >> 2))/div;
		if (res > max)
			break;
		old = res;
		printf("%u\n", res);
		i++;
	}
	i--;
_break:
	
	printf("%hu %u\n", prescalers[i].fact, max - old);
}

void usage(const char *name)
{
	fprintf(stderr, "%s FREQ MAX\n", name);
	exit(1);
}

int main(int argc, char *argv[])
{
	unsigned int a;
	unsigned short b;

	if (argc != 3)
		usage(*argv);
	
	if (!sscanf(argv[1], "%u", &a)
		|| !sscanf(argv[2], "%hu", &b))
		usage(*argv);
	
	select_prescaler(a, b);

	return 0;
}
