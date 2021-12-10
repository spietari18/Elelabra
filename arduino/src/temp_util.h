#ifndef TEMP_UTIL_H
#define TEMP_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* Monta kalibrointipistettä voi
 * olla enintään muistissa.
 */
#define MAX_POINTS 32

/* Näytepuskurin koko. (mitä isompi, sen parempi suodatus) */
#define MAX_SAMPLES 128

/* AD muuntimen näytteistystaajuus. */
#define SAMPLE_RATE 64 // [Hz]

/* Kuinka paljon näytteet saavat enintään
 * muuttua näytepuskurin moodista.
 */
#define SAMPLE_MAX_DELTA 4.0

/* Absoluuttiset rajat lämpötilalle.
 * (määrittyy lähinnä vahvistinkytkennästä)
 */
#define T_ABS_MIN -50.0
#define T_ABS_MAX  80.0

/* Aseta sisäänrakennetut kalibrointipisteet. */
void default_points();

/* Laske PNS kertoimet. */
void compute_lss_coefs();

/* Lue uusi lämpötila. */
float temp_update();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // !TEMP_UTIL_H
