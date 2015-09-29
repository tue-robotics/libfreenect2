#ifndef APPROX_ATAN2_H_
#define APPROX_ATAN2_H_

#include <cmath>

#define ATAN2_APPROX_N 10000

class Atan2
{

public:

    void initialize()
    {
        slope_factor_ = ATAN2_APPROX_N - 1;

        // We divide the unit circle into 8 parts, split by 3 criteria:
        // abs(x) < abs(y)
        // x < 0
        // y < 0

        // This way we can always calculate a slope between 0 and 1 (by dividing the
        // smallest coordinate by the biggest).
        for(int j = 0; j < 8; ++j)
        {
            for(int k = 0; k < ATAN2_APPROX_N; ++k)
            {
                // Calculate the slope corresponding to this table entry
                double slope = (double)k / slope_factor_;

                // Calculate a virtual (x, y) cartesian point which corresponds to this table entry
                // based on the slope. Determine the part of the unit circle in which (x, y) lies
                // base on j
                double x, y;
                if (j&1)
                {
                    // x_abs > y_abs
                    x = 1;
                    y = slope + 1e-16; // Necessary because of 0-check in getAngle
                }
                else
                {
                    // x_abs < y_abs
                    x = slope + 1e-16; // Necessary because of 0-check in getAngle
                    y = 1;
                }

                if (!(j&2))
                {
                    // x < 0
                    x = -x;
                }

                if (!(j&4))
                {
                    // y < 0
                    y = -y;
                }

                table_[j][k] = std::atan2(y, x);
            }
        }
    }

    inline float atan2(float y, float x)
    {
        if (x == 0 && y == 0)
            return 0;

        int j; // Circle region number (0 - 7)

        float x_abs;
        if (x < 0)
        {
            j = 0;
            x_abs = -x;
        }
        else
        {
            j = 2;
            x_abs = x;
        }

        float y_abs;
        if (y < 0)
        {
            y_abs = -y;
        }
        else
        {
            j += 4;
            y_abs = y;
        }

        // Calculcate slope

        float slope;
        if (x_abs < y_abs)
        {
            slope = x_abs / y_abs;
        }
        else
        {
            slope = y_abs / x_abs;
            j += 1;
        }

        int k = slope_factor_ * slope;

        return table_[j][k];
    }

private:

    float slope_factor_;

    float table_[8][ATAN2_APPROX_N];
};

#endif
