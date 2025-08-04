#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358
#endif

int main(void)
{
  FILE* f = fopen("sinewave.pcm", "wb");
  double t;
  for (t = 0; t < 1; t += 1./44100) // 8000 is the sample rate in Hz
  {
    double sample = 15000 * sin(2 * M_PI * 440 * t); // 1000 Hz sine wave
    short s16 = (short)sample;
    unsigned char c;
    c = (unsigned)s16 % 256;
    fwrite(&c, 1, 1, f);
    c = (unsigned)s16 / 256 % 256;
    fwrite(&c, 1, 1, f);
  }
  fclose(f);
  return 0;
}
