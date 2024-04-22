struct LRPower {
    float lPower;
    float rPower;
};

LRPower inverse(float throttle, float theta){
    LRPower lrPower;
    lrPower.lPower = throttle - theta;
    lrPower.rPower = throttle + theta;

    return lrPower;
}



