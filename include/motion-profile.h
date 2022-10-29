#include "vex.h"
#include <math.h>
#include <iostream>
#include <vector>


const float dT = 10;

#define MAX_V 1.56


float linearConversion(float velocityValue)
{
  //return velocityValue*(60/ (.0254 * (M_PI * 3.25) ) );
  //return (300 * velocityValue)/.78;
  return velocityValue / MAX_V * 600;
}

std::vector<float> profileGeneration(float targetDistance)
{

  //constants
  const float maxVel = 1.56;
  const float accelStep = 2.0;
  //const float startSpeed = 0;

  float accelTime = maxVel / accelStep;
  float cruiseTime =  ( targetDistance - (accelTime * maxVel) ) / maxVel;
  
  
  //time start
  float currentTime = 0;
  float velocityVal;
  float acc;
  float previousVel = 0;
  float finalVel;
  std::vector<float> velocityVector;
  while(currentTime < 2*accelTime + cruiseTime)
  {
    //accel
    if(currentTime < accelTime)
    {
      acc = accelStep;
      // velocityVal = currentTime * accelStep;
      // velocityVector.push_back(velocityVal);
    }
    //cruise
    else if(currentTime > accelTime && currentTime < accelTime + cruiseTime)
    {
      acc = 0;
      // velocityVal = maxVel;
      // velocityVector.push_back(velocityVal);
    }
    //decel
    else if(currentTime > accelTime + cruiseTime && currentTime < 2*accelTime + cruiseTime)
    {
      acc = -accelStep;
      //velocityVal = maxVel - (currentTime-accelTime - cruiseTime)*accelStep;
      //velocityVector.push_back(velocityVal);
    }
    finalVel = previousVel + (acc * (dT/1000) ) ;

    velocityVector.push_back(finalVel);
    previousVel = finalVel;
    //wait( dT, msec);
    currentTime += 0.01;
  }

  velocityVector.push_back(0);
  return velocityVector;

}

void profileFollow(std::vector<float> velocityValues)
{

  for(int i = 0; i < velocityValues.size(); i++)
  {
    //Controller1.Screen.print(velocityValues[i]);
    float convertedVelocity = linearConversion(velocityValues[i] );


    FrontLeft.spin(forward, convertedVelocity, velocityUnits::rpm);
    MiddleLeft.spin(forward, convertedVelocity, velocityUnits::rpm);
    BackLeft.spin(forward, convertedVelocity, velocityUnits::rpm);
    FrontRight.spin(forward, convertedVelocity, velocityUnits::rpm);
    MiddleRight.spin(forward, convertedVelocity, velocityUnits::rpm);
    BackRight.spin(forward, convertedVelocity, velocityUnits::rpm);
        

    wait(dT, msec);
    //Controller1.Screen.clearScreen();
  }
  FrontLeft.stop(brake);
  MiddleLeft.stop(brake);
  BackLeft.stop(brake);
  FrontRight.stop(brake);
  MiddleRight.stop(brake);
  BackRight.stop(brake);
 // wait(5, sec);
      //BackRight.spin(reverse, 200, velocityUnits::rpm);



}



