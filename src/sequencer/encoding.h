#ifndef ENCODING_H
#define ENCODING_H

/* Linear quantization helpers — pure functions, no side-effects, no globals */

int   float_to_uint8 (float x, float x_min, float x_max);
int   float_to_uint12(float x, float x_min, float x_max);
int   float_to_uint16(float x, float x_min, float x_max);

float uint8_to_float (int x_int, float x_min, float x_max);
float uint12_to_float(int x_int, float x_min, float x_max);
float uint16_to_float(int x_int, float x_min, float x_max);

#endif /* ENCODING_H */
