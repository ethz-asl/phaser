#pragma once

#include <array>
#include <cmath>

namespace common {

const int pixels_per_column = 64;
const int columns_per_buffer = 16;

const int pixel_bytes = 12;
const int column_bytes = 16 + (pixels_per_column * pixel_bytes) + 4;

const int encoder_ticks_per_rev = 90112;

const std::array<float, pixels_per_column> beam_altitude_angles = {
   16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
   12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
   8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
   3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
   -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
   -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
   -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
   -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
};
const std::array<float, pixels_per_column> beam_azimuth_angles = {
   3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
   3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
   3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
   3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
   3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
   3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
   3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
   3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
};

struct trig_table_entry {
   float sin_beam_altitude_angles;
   float cos_beam_altitude_angles;
   float beam_azimuth_angles;
};

// table of vertical angle cos, sin, and horizontal offset of each pixel
static trig_table_entry trig_table[pixels_per_column];

static bool init_tables() {
   for (int i = 0; i < pixels_per_column; i++) {
       trig_table[i] = {sinf(beam_altitude_angles[i] * 2 * M_PI / 360.0f),
                        cosf(beam_altitude_angles[i] * 2 * M_PI / 360.0f),
                        beam_azimuth_angles[i] * 2 * (float)M_PI / 360.0f};
   }
   return true;
}

} // namespace common
