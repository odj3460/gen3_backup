
#ifndef _LowPassFilter_hpp_
#define _LowPassFilter_hpp_

#include <cmath>

class LowPassFilter_3{
public:
	//constructors
	LowPassFilter_3();
	LowPassFilter_3(float iCutOffFrequency, float iDeltaTime);
	//functions
	float update(float input);
	float update(float input, float deltaTime, float cutoffFrequency);
	//get and configure funtions
	float getOutput() const{return output;}
	void reconfigureFilter(float deltaTime, float cutoffFrequency);
private:
	float output;
	float ePow;
};

#endif //_LowPassFilter_hpp_
