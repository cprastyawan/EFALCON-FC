#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
void getQ(float * buf);

#endif // _QUATERNIONFILTERS_H_
