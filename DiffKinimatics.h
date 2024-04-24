#ifndef DIFFKINIMATICS_H
#define DIFFKINIMATICS_H

struct LRPower
{
    float lPower;
    float rPower;
};

LRPower inverse(float throttle, float theta);

#endif // DIFFKINIMATICS_H