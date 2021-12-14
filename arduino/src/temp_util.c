#include "pins.h"
#include "util.h"
#include "macro.h"
#include "temp_util.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

/* kalibrointipisteet */
static uint8_t num_points;
static float points[MAX_POINTS][2];

/* sisäänrakennetut kalibrointipisteet
 * (Laitettu ROM:iin koska säästää RAM muistia)
 */
static const float points_builtin[][2] PROGMEM = {
	/* S, T */
	{684.0, -51.70},
	{2496.0, 79.76}
};

/* PNS kertoimet */
static float lss_coefs[2];

/* AD muuntimelta luetut näytteet */
static uint8_t sample_pos;
static float samples[MAX_SAMPLES];

/* Laske PNS kertoimet. */
static void compute_lss_coefs()
{
	float Sx = 0, Sy = 0, Sxx = 0, Sxy = 0, Syy = 0;

	for (uint8_t i = 0; i < num_points; i++)
	{
		Sx  += points[i][0];
		Sy  += points[i][1];
		Sxy += points[i][0]*points[i][1];
		Sxx += points[i][0]*points[i][0];
		Syy += points[i][1]*points[i][1];
	}

	lss_coefs[0] = (Sxx*Sy - Sx*Sxy)/(num_points*Sxx - Sx*Sx);
	lss_coefs[1] = (num_points*Sxy - Sx*Sy)/(num_points*Sxx - Sx*Sx);
}

/* Aseta sisäänrakennetut kalibrointipisteet. */
void default_points()
{
	num_points = sizeof(points_builtin)/sizeof(points_builtin[0]);
	(void)memcpy_P(points, points_builtin, sizeof(points_builtin));
	compute_lss_coefs();
}

/* Lue raaka näyte AD-muuntimelta. */
uint16_t read_sample_raw()
{
	uint16_t tmp = 0;

	/* lue näyte AD-muuntimelta */
	WRITE(SPI_SS, LOW);
	(void)spi_byte(0b00000001);
	tmp |= (uint16_t)spi_byte(0b00100000) << 8;
	tmp |= spi_byte(0);
	WRITE(SPI_SS, HIGH);

	/* näyte on 12 bittiä */
	return tmp & ((1 << 12) - 1);
}

static int float_cmp(const void *a, const void *b)
{
	return *(float *)a - *(float *)b;
}

/* Lue suodatettu näyte AD-muuntimelta. */
float read_sample()
{
	float buffer[MAX_SAMPLES];
	float tmp;

	/* lue näyte kiertopuskuriin */
	samples[sample_pos] = read_sample_raw();
	INC_MOD(sample_pos, MAX_SAMPLES);

	/* kopioi puskuri */
	(void)memcpy(buffer, samples, MAX_SAMPLES*sizeof(samples[0]));

	/* järjestä puskuri numerojärkestykseen */
	qsort(buffer, MAX_SAMPLES, sizeof(buffer[0]), &float_cmp);

	/* laske kiertopuskurin moodi */
#if (MAX_SAMPLES & 1)
	tmp = buffer[MAX_SAMPLES/2];
#else
	tmp = (buffer[MAX_SAMPLES/2] + buffer[MAX_SAMPLES/2 + 1])/2;
#endif

	/* suodata pois arvot, jotka ovat moodia SAMPLE_MAX_DELTA
	 * suurempia tai pienempiä. (poistaa äkilliset vaihtelut)
	 */
	uint8_t beg = 0, end = MAX_SAMPLES - 1;
	while ((beg < MAX_SAMPLES/2) &&
		(abs(tmp - buffer[beg]) > SAMPLE_MAX_DELTA))
		beg++;
	while ((end > MAX_SAMPLES/2) &&
		(abs(tmp - buffer[end]) > SAMPLE_MAX_DELTA))
		end--;

	/* laske puskurin keskiarvo */
	tmp = 0;
	for (uint8_t i = beg; i <= end; i++)
		tmp += buffer[i];
	tmp /= end - beg + 1;

	return tmp;
}

/* Lue lämpötila AD-muuntimelta. */
float read_temp()
{
	/* lineaarinen PNS sovitus */
	return lss_coefs[0] + lss_coefs[1]*read_sample();
}
