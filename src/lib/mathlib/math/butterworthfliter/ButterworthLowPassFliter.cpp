#include <px4_defines.h>
#include "ButterworthLowPassFliter.hpp"
#include <cmath>

namespace math
{

    float ButterworthLowPassFliter::butterworth_lowpass_fliter(float raw_input, int frequency, int fliter_hz)   //fliter frequency select:20hz,25hz,30hz,35hz
    {	   
        if(last_fliter_hz != fliter_hz)
        {
            if(fliter_hz == 20)
            {
                fliter_select = 0;
            }
            else if(fliter_hz == 25)
            {
                fliter_select = 1;
            }
            else if(fliter_hz == 30)
            {
                fliter_select = 2;
            }
            else
            {
                fliter_select = 3;
            }
            for(int i = 0; i < 5; i++)
            {
                if(frequency == 800)
                {
                    fliter_param[i] = butterworth_lowpass_param_800[fliter_select][i];
                }
                else
                {
                    fliter_param[i] = butterworth_lowpass_param_1000[fliter_select][i];
                }
            }
        }
        last_fliter_hz = fliter_hz;
    
        fliter_output = fliter_param[0]*raw_input + fliter_param[1]*last_input_param[0] + fliter_param[2]*last_input_param[1]
                        + fliter_param[3]*last_output_param[0] + fliter_param[4]*last_output_param[1];
        last_input_param[1] = last_input_param[0];
        last_input_param[0] = raw_input;
        last_output_param[1] = last_output_param[0];
        last_output_param[0] = fliter_output;
    
        return fliter_output;
    }
}