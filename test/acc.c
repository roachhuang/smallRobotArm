// function to return the next delay value for positive acceleration
float positiveAcceleration(float waitTime)
{
    float dVelocity = waitTime * accelerationTerm;
    waitTime = 1 / (dVelocity + 1 / waitTime);
    // if (waitTime < 0.00025) {
    // waitTime = 0.00025;
    // }
    return waitTime;
}
main()
{
    const float UNIT_TD = 0.05625; // this timedelay is for one deg per sec.

    // float waitTime = (PI/(6400*36*(PI/180))); //36deg/s -> (180/(6400*36))*1000000 = 781 microsec. the largger the time is, the slower the motor run.
    float waitTime = UNIT_TD / 18.0;                           // UNIT_TD / 3.6;  //set init vel as 36deg /s
    float accelerationTerm = (UNIT_TD / 0.000002) * 1000000.0; // (UNIT_TD/1.2)*1000000;  // set acc as 12deg/s^2, the time delay for it will be acclerationTerm.
    int roundedWaitTime;
    for (int x = 0; x < 6400; x++)
    {
        waitTime = positiveAcceleration(waitTime);
        roundedWaitTime = round(waitTime * 1000000);
        // digitalWrite(PUL3_PIN, HIGH);
        // delayMicroseconds(roundedWaitTime);
        // digitalWrite(PUL3_PIN, LOW);
        // delayMicroseconds(roundedWaitTime);
    }
}
