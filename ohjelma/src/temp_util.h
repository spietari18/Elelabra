#ifndef TEMP_UTIL_H
#define TEMP_UTIL_H

/* Monta kalibrointipistettä voi
 * olla enintään muistissa.
 */
#define MAX_POINTS 32

/* Näytepuskurin koko. (mitä isompi, sen parempi suodatus) */
#define MAX_SAMPLES 64

/* AD muuntimen näytteistystaajuus. */
#define SAMPLE_RATE 64 // [Hz]

/* Kuinka paljon näytteet saavat enintään
 * muuttua näytepuskurin mediaani.
 */
#define SAMPLE_MAX_DELTA 4.0

/* Absoluuttiset rajat lämpötilalle.
 * (määrittyy lähinnä vahvistinkytkennästä)
 */
#define T_ABS_MIN -50.0
#define T_ABS_MAX  80.0

/* nämä ovat globaaleja, jotta niitä voi manipuloida
 * temp_util.c:n ulkopuolella
 */
extern uint8_t n_data_points;
extern float data_points[MAX_POINTS][2];

/* Laske PNS kertoimet. */
void compute_lss_coefs();

/* Aseta sisäänrakennetut kalibrointipisteet. */
void default_points();

/* Lue suodatettu näyte AD-muuntimelta. */
float read_sample();

/* PNS sovitus. */
float calc_temp(uint16_t);

/* Lue lämpötila AD-muuntimelta. */
float read_temp();

#endif // !TEMP_UTIL_H
