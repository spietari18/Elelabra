#include "pins.h"
#include "util.h"
#include "macro.h"
#include "temp_util.h"

#include <string.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/pgmspace.h>

/* kalibrointipisteet */
uint8_t n_data_points;
float data_points[MAX_POINTS][2];

/* sisäänrakennetut kalibrointipisteet
 * (Laitettu ROM:iin koska säästää RAM muistia)
 */
static const float points_builtin[][2] PROGMEM = {
	/* S, T */
	{1110.0, -05.04},
	{2015.0,  46.47}
};

/* PNS kertoimet */
static float lss_coefs[2];

/* AD muuntimelta luetut näytteet */
static uint8_t sample_pos;
static float samples[MAX_SAMPLES];

/* Laske PNS kertoimet. */
void compute_lss_coefs()
{
	float Sx = 0, Sy = 0, Sxx = 0, Sxy = 0, Syy = 0;

	for (uint8_t i = 0; i < n_data_points; i++)
	{
		Sx  += data_points[i][0];
		Sy  += data_points[i][1];
		Sxy += data_points[i][0]*data_points[i][1];
		Sxx += data_points[i][0]*data_points[i][0];
		Syy += data_points[i][1]*data_points[i][1];
	}

	lss_coefs[0] = (Sxx*Sy - Sx*Sxy)/(n_data_points*Sxx - Sx*Sx);
	lss_coefs[1] = (n_data_points*Sxy - Sx*Sy)/(n_data_points*Sxx - Sx*Sx);
}

/* Aseta sisäänrakennetut kalibrointipisteet. */
void default_points()
{
	n_data_points = sizeof(points_builtin)/sizeof(points_builtin[0]);
	(void)memcpy_P(data_points, points_builtin, sizeof(points_builtin));
	compute_lss_coefs();
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
	samples[sample_pos] = adc_sample(ADC_CH0);
	INC_MOD(sample_pos, MAX_SAMPLES);

	/* kopioi puskuri */
	(void)memcpy(buffer, samples, MAX_SAMPLES*sizeof(samples[0]));

	/* järjestä puskuri numerojärkestykseen */
	qsort(buffer, MAX_SAMPLES, sizeof(buffer[0]), &float_cmp);

	/* laske kiertopuskurin mediaani */
#if (MAX_SAMPLES & 1)
	tmp = buffer[MAX_SAMPLES/2];
#else
	tmp = (buffer[MAX_SAMPLES/2] + buffer[MAX_SAMPLES/2 + 1])/2;
#endif

	/* suodata pois arvot, jotka ovat mediaania SAMPLE_MAX_DELTA
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

/* PNS sovitus. */
float calc_temp(uint16_t S)
{
	return lss_coefs[0] + lss_coefs[1]*S;
}

/* Lue lämpötila AD-muuntimelta. */
float read_temp()
{
	return calc_temp(read_sample());
}
