#include <stdio.h>

/**
 * Calibrate SICK DT35
 *
 * dead_adc : ADC count in the dead-zone (the one you always input)
 * adc[]    : array of measured ADC counts at known distances
 * dist[]   : array of those true distances (meters)
 * N        : number of samples in adc[]/dist[]
 *
 * Outputs via pointers:
 *   *offset = ADC offset to subtract  (will equal dead_adc)
 *   *scale  = counts per meter
 *   *base   = dead-zone distance in meters
 *
 * Returns 0 on success, –1 on error (e.g. N<2 or singular fit).
 */
int calibrate_dt35(int    dead_adc,
                   int    N,
                   float  adc[],
                   float  dist[],
                   float *offset,
                   float *scale,
                   float *base)
{
    if (N < 2) return -1;

    // 1) Offset is simply the dead-zone ADC
    *offset = (float)dead_adc;

    // 2) Fit dist = a*adc + c  via least squares on all N points
    double sum_x  = 0.0, sum_y  = 0.0;
    double sum_xy = 0.0, sum_x2 = 0.0;
    for (int i = 0; i < N; i++) {
        double x = (double)adc[i];
        double y = (double)dist[i];
        sum_x  += x;
        sum_y  += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }
    double M     = (double)N;
    double denom = M * sum_x2 - sum_x * sum_x;
    if (denom == 0.0) return -1;

    // a = slope = 1/scale,  c = intercept
    double a = (M * sum_xy - sum_x * sum_y) / denom;
    double c = (sum_y - a * sum_x)     / M;

    // 3) Recover scale and base
    *scale = 1.0f / (float)a;
    // base = c + dead_adc/scale  == c + dead_adc * a
    *base  = (float)(c + (double)dead_adc * a);

    return 0;
}

int main(void) {
    // Suppose you measured at three distances:
    //   dead_adc = 45   (unknown base)
    //   (adc,dist) = (200, 0.30), (600, 0.80), (1000, 1.40)
    int   dead_adc = 37;
    float adc_samples[]  = {6144, 4420, 2650, 1665};
    float dist_samples[] = {2.80, 2.02, 1.22, 0.78};
    int   N = sizeof(adc_samples)/sizeof(adc_samples[0]);

    float offset, scale, base;
    if (calibrate_dt35(dead_adc, N, adc_samples, dist_samples,
                       &offset, &scale, &base) == 0)
    {
        printf("Calibration results:\n");
        printf("  offset = %.4f  // ADC counts to subtract\n", offset);
        printf("  scale  = %.4f  // counts per meter\n", scale);
        printf("  base   = %.4f  // dead-zone distance (m)\n", base);
    } else {
        fprintf(stderr, "Calibration failed (insufficient data or singular fit)\n");
    }
    return 0;
}