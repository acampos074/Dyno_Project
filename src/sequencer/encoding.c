#include "encoding.h"

int float_to_uint8(float x, float x_min, float x_max)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * (float)((1 << 8) - 1) / span);
}

int float_to_uint12(float x, float x_min, float x_max)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * (float)((1 << 12) - 1) / span);
}

int float_to_uint16(float x, float x_min, float x_max)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * (float)((1 << 16) - 1) / span);
}

float uint8_to_float(int x_int, float x_min, float x_max)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / (float)((1 << 8) - 1) + offset;
}

float uint12_to_float(int x_int, float x_min, float x_max)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / (float)((1 << 12) - 1) + offset;
}

float uint16_to_float(int x_int, float x_min, float x_max)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / (float)((1 << 16) - 1) + offset;
}
