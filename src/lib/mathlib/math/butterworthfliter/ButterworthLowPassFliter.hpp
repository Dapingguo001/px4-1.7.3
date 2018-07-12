#pragma once

namespace math
{
class ButterworthLowPassFliter
{
public:
   	float butterworth_lowpass_fliter(float raw_input, int frequency, int fliter_hz);   //fliter frequency select:20hz,25hz,30hz,35hz

private:

    float fliter_param[5];
    float fliter_output;
    int fliter_select;
    int last_fliter_hz = 0;
    float last_input_param[2] = {0.0,0.0};
	float last_output_param[2] = {0.0,0.0};
	float butterworth_lowpass_param_1000[4][5] = {{0.00362,0.00724,0.00362,1.82270,-0.83720},   //1000_20hz lowpass fliter
											      {0.00554,0.01108,0.00554,1.77863,-0.80080},   //1000_25hz lowpass fliter
											      {0.00782,0.01564,0.00782,1.73472,-0.76601},   //1000_30hz lowpass fliter
                                                  {0.01043,0.02086,0.01043,1.69100,-0.73273}};  //1000_35hz lowpass fliter
                                             
    float butterworth_lowpass_param_800[4][5] = {{0.00554,0.01108,0.00554,1.77863,-0.80080},     //800_20hz lowpass fliter
                                                {0.00844,0.01688,0.00844,1.72378,-0.75755},     //800_25hz lowpass fliter
                                                {0.01186,0.02372,0.01186,1.66920,-0.71663},     //800_30hz lowpass fliter
                                                {0.01575,0.03150,0.01575,1.61494,-0.67794}};    //800_35hz lowpass fliter
};

} // namespace math