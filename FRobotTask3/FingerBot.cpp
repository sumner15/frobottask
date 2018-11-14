#include "StdAfx.h"
#include "FingerBot.h"

#include <string>
#include <iostream>
#include <cmath>

const double  FingerBot::KpRange[2] = {-1.0, 20.0};
const double  FingerBot::KdRange[2] = {-0.3, 1};

// the are probably machine dependant, so it is dumb for us to drop them here as if they are reliable constants - Just thought we should let you know that we know that.                                                                                                                                                                                                                                                             have a nice day.
const double FingerBot::leftOrientAccel  = 0.08583;  
const double FingerBot::rightOrientAccel = 0.01418;

FingerBot::FingerBot(std::string ipAddress, std::string ipPort, std::string mdlName) : XPCModel(ipAddress, ipPort, mdlName){
	
	std::cout << FingerBot::KdRange[0] << std::endl;

	loadIdMap();
	//setHandMode("left"); //if wanted, this function sets handedness manually...
	autoSetHandMode(); //...otherwise, handedness is set automatically.
}

/**---------------------------- set forces on/off -----------------------------//
*/
void FingerBot::setForcesOn(bool forceOn){
    if (forceOn){
        setParamByName("forceOn", 1);
	} else {
        setParamByName("forceOn", 0);
	}     
}

/**------------------------- set handedness of robot --------------------------//
*/
void FingerBot::setHandMode(std::string handedness){       
    std::cout << "handedness set to " << handedness << std::endl;
	if (handedness.compare("right")){
        setParamByName("leftMode", 0);
        rightHandMode = true;
	} else if (handedness.compare("left")){
        setParamByName("leftMode", 1);
        rightHandMode = false;
	} else {
        std::cout << "the value passed to the setHandMode function can only be left or right" << std::endl;
	}
}

/**------------------------- set Finger Orientation ---------------------------//
* use the accelerometers to determine handedness.
*/
void FingerBot::autoSetHandMode(){
    double distL = abs(getSignalByName("orientAccel") - FingerBot::leftOrientAccel);
    double distR = abs(getSignalByName("orientAccel") - FingerBot::rightOrientAccel);
	std::cout << "distL: " << distL << "\t" << "distR: " << distR << std::endl;
	std::cout << "accel orientation: " << getSignalByName("orientAccel") << std::endl;
    if (distL < distR){
        setParamByName("leftMode", 1);
        rightHandMode = false;
	}else{
        setParamByName("leftMode", 0);
        rightHandMode = true;
        std::cout << "set to right mode" << std::endl;
	}
}


/**----------------------------- set movement duration ------------------------//
'' duration argument is in miliseconds
*/
void FingerBot::setMovementDuration(double dur){
    setParamByName("fixedDur", dur);
}

/**---------------------------- set hit times ---------------------------------//
fNum 0 means move top finger. fnum 1 means move bottom finger. fNum 2 means move 
both fingers.
*/
void FingerBot::setHitTimes(double hitTime,int fNum){
    upcomingMove.fNum = fNum;
	upcomingMove.time = hitTime;
	switch(fNum){
	case 0:
        setParamByName("hitTime1", (hitTime));
		break;
	case 1:
        setParamByName("hitTime2", (hitTime));
		break;
	case 2:
        setParamByName("hitTime1", (hitTime));
        setParamByName("hitTime2", (hitTime));
		break;
	}
	//std::cout << "set hit time to " << hitTime << "for finger code " << fNum << std::endl;
}

/**-------------------------- change the target position ----------------------//
setHitPos sets the endpoint of the robots motion
0 is defined as no motions 
1 is defined as the full range of motion
*/
void FingerBot::setHitPos(double pos,int fNum ){
    if (pos < 0 || pos > 1) {           
		throw new std::exception("the desired position is out of range -  not set");
	}else{
        if (fNum == 0){
            setParamByName("hitPos1", pos);
		}else if (fNum == 1){
            setParamByName("hitPos2", pos);
		}else if (fNum == 2){
            setParamByName("hitPos1", pos);
            setParamByName("hitPos2", pos);
		}else{            
			throw new std::exception("the 'finger' argument must be either 0 (index) 1 (middle) or 2 (both)");
		}
	}
}

/**------------------------ turon auto mode on and off ------------------------//
setting trajMode to:
1 will put us in subject-initiated trajectory mode.
2 will put us in auto trajectory mode
3 will put us in auto trajectory mode without extensions (just moves to a point)
*/
void FingerBot::setTrajMode(double setting){
    if (setting < 1 || setting > 4){
        std::cout << "trajectory mode: " << setting << std::endl;
		throw new std::exception("bad trajectory mode set. Trajectory mode must be 1, 2, 3 or 4");
	}
    setParamByName("trajMode", setting);
	std::cout << "set trajectory mode to " << setting << std::endl;
}

/**------------------------ set mode 1 parameters ------------------------//
velocity threshold: v_thresh
force theshold: f_thresh
maximum trajectory duration: max_traj_dur
*/
void FingerBot::setVThresh(double setting){    
    setParamByName("v_thresh", setting);
}
void FingerBot::setFThresh(double setting){
	setParamByName("f_thresh", setting);
}
void FingerBot::setMaxTrajDur(double setting){
	setParamByName("maxTrajDur", (setting));
}


/**--------------------- set the proportional gains to zero -------------------//
*/
void FingerBot::zeroPropGains(){
    setKp(0.0, 3);
}

/**--------------------- set the differential gains to zero -------------------//
*/
void FingerBot::zeroDifGains(){
    setKd(0.0, 3);
}

/**--------------------------- set Kd gains together --------------------------//
*/
void FingerBot::setKp(double kp, int finger){    
	switch (finger){
	case 0:
		setParamByName("Kp1", kp);
		break;
	case 1:
		setParamByName("Kp2", kp);
		break;
	case 2:
		setParamByName("Kp1", kp);
		setParamByName("Kp2", kp);
		break;
	}
    std::cout << "kp set" << std::endl;
}

/**--------------------------- set Kd gains together --------------------------//
*/
void FingerBot::setKd(double kd, int finger){
    
	switch (finger){
	case 0:
		setParamByName("Kd1", kd);
		break;
	case 1:
		setParamByName("Kd2", kd);
		break;
	case 2:
		setParamByName("Kd1", kd);
		setParamByName("Kd2", kd);
		break;
	}

    std::cout << "kd set" << std::endl;
}

/**------------------------- check gains in range -----------------------------//
checks if a specified gain value is within our specified range
*/
std::string FingerBot::checkGainInRange(double val, bool propGain){
    bool withinUpper = false;
    bool aboveLower  = false;
    if (propGain) {
		if (val >= FingerBot::KpRange[0]){ aboveLower = true; }
		if (val <= FingerBot::KpRange[1]){ withinUpper = true; }
	} else {
		if (val >= FingerBot::KdRange[0]){ aboveLower = true; }
		if (val <= FingerBot::KdRange[1]){ withinUpper = true; }
    }

	if (withinUpper && aboveLower) { return "good"; }
	if (withinUpper && !aboveLower) {return "tooLow"; }
	if (!withinUpper && aboveLower) {return "tooHigh"; }
	else { return "fail"; }
}

/**---------------------------- set gainChange Rate ---------------------------//
* sets the amount of time it takes to change from one gain value to another - 
* this is not instantaneous because the controller does not like discontinuities.
*/
void FingerBot::setGainChangeRate(double rate){
    if (rate >= 0.001){
        setParamByName("changeRate", rate);
	} else {
        throw new std::exception("value for gain change rate is too low - value not set.");        
    }
}

/**------------------- get feedback on time to threshold ----------------------//
*/
double FingerBot::getFeedback(int fNum){
	if (fNum == 0) {
        return getSignalByName("feedback1");
    } else if (fNum == 1) {
        return getSignalByName("feedback2");
    } else if (fNum == 2) {
        return (getSignalByName("feedback1")+getSignalByName("feedback2"))/2;        
    }
    
}

/**------------------------------- get prop gains -----------------------------//
*/
void FingerBot::getPropGains(double& kp1, double& kp2){
    kp1 = getSignalByName("Kp1");
    kp2 = getSignalByName("Kp2");
}

/**-------------------------- get the robot positions -------------------------//
*/
double FingerBot::getPos1(){
	return getSignalByName("pos1");	
}
double FingerBot::getPos2(){
	return getSignalByName("pos2");	
}

/**-------------------------- get the robot forces ----------------------------//
*/
double FingerBot::getForce1(){
	return getSignalByName("force1");	
}
double FingerBot::getForce2(){
	return getSignalByName("force2");	
}

/**----------------------- set the purse viscosity gains ----------------------//
*/
void FingerBot::setViscGains(double gain, int fNum){
    if (fNum == 0) {
        setParamByName("KdV1", gain);
    } else if (fNum == 1) {
        setParamByName("KdV2", gain);
    } else if (fNum == 2) {
        setParamByName("KdV1", gain);
        setParamByName("KdV2", gain);
    }
}

/** ---------------------------- get target time ------------------------------//
*/
double FingerBot::getTargetTime(){
	double currentTargetTime = getSignalByName("targetTime");
	return currentTargetTime;
}

/**------------------------- get parameter from name --------------------------//
*/
double FingerBot::getParamByName(std::string paramName){
	return getParamById(paramMap[paramName]);
}

/**------------------------- set parameter from name --------------------------//
*/
void FingerBot::setParamByName(std::string paramName, double paramValue){
	setParamById(paramMap[paramName], paramValue);
	std::cout << paramName << " set to " << getParamByName(paramName) << std::endl;
}

/**------------------------- get signal from name -----------------------------//
*/
double FingerBot::getSignalByName(std::string signalName){
	return getSignalById(signalMap[signalName]);
}

/**--------------------------------- load id map  -----------------------------//
* 
*/
void FingerBot::loadIdMap(){			    
    signalMap.insert(std::make_pair("pos1", findSignal("signals/sigPos1")));
    signalMap.insert(std::make_pair("pos2", findSignal("signals/sigPos2")));
    signalMap.insert(std::make_pair("vel1", findSignal("signals/sigVel1")));
    signalMap.insert(std::make_pair("vel2", findSignal("signals/sigVel2")));
    signalMap.insert(std::make_pair("pos1Des", findSignal("signals/sigPos1Des")));
    signalMap.insert(std::make_pair("pos2Des", findSignal("signals/sigPos2Des")));
    signalMap.insert(std::make_pair("vel1Des", findSignal("signals/sigVel1Des")));
    signalMap.insert(std::make_pair("vel2Des", findSignal("signals/sigVel2Des")));
    signalMap.insert(std::make_pair("checkHand", findSignal("parameters/parLeftMode")));
    signalMap.insert(std::make_pair("targetTime", findSignal("signals/sigTargetTime")));
    signalMap.insert(std::make_pair("Kp1", findSignal("parameters/parKp1")));
    signalMap.insert(std::make_pair("Kp2", findSignal("parameters/parKp2")));
    signalMap.insert(std::make_pair("fixedDur", findSignal("parameters/parFixedDur")));
    signalMap.insert(std::make_pair("hitTime1", findSignal("trajectory params/parTHit1")));
    signalMap.insert(std::make_pair("hitTime2", findSignal("trajectory params/parTHit2")));
    signalMap.insert(std::make_pair("hitPos1", findSignal("trajectory params/parP1Des")));
    signalMap.insert(std::make_pair("hitPos2", findSignal("trajectory params/parP2Des")));
    signalMap.insert(std::make_pair("force1", findSignal("signals/sigForce1")));
    signalMap.insert(std::make_pair("force2", findSignal("signals/sigForce2")));
    signalMap.insert(std::make_pair("force1Clean", findSignal("signals/sigForce1Clean")));
    signalMap.insert(std::make_pair("force2Clean", findSignal("signals/sigForce2Clean")));
    signalMap.insert(std::make_pair("marker", findSignal("signals/sigMarker")));
    signalMap.insert(std::make_pair("lcF1a", findSignal("signals/sigLoadCF1a")));
    signalMap.insert(std::make_pair("lcF1b", findSignal("signals/sigLoadCF1b")));
    signalMap.insert(std::make_pair("lcF2a", findSignal("signals/sigLoadCF2a")));
    signalMap.insert(std::make_pair("lcF2b", findSignal("signals/sigLoadCF2b")));
    signalMap.insert(std::make_pair("lcReference", findSignal("signals/sigLoadC")));
    signalMap.insert(std::make_pair("orientAccel", findSignal("signals/sigGravAccel")));	
	signalMap.insert(std::make_pair("feedback1",findSignal("signals/sigTimeToThresh1")));
	signalMap.insert(std::make_pair("feedback2",findSignal("signals/sigTimeToThresh2")));

	paramMap.insert(std::make_pair("v_thresh",findParam("parameters/parVThresh", "Gain")));
    paramMap.insert(std::make_pair("f_thresh",findParam("parameters/parFThresh", "Gain")));
    paramMap.insert(std::make_pair("Kp1",findParam("parameters/parKp1", "Gain")));
    paramMap.insert(std::make_pair("Kp2",findParam("parameters/parKp2", "Gain")));
    paramMap.insert(std::make_pair("Kd1",findParam("parameters/parKd1", "Gain")));
    paramMap.insert(std::make_pair("Kd2",findParam("parameters/parKd2", "Gain")));
    paramMap.insert(std::make_pair("KdV1",findParam("parameters/parKdV1", "Gain")));
    paramMap.insert(std::make_pair("KdV2",findParam("parameters/parKdV2", "Gain")));
    paramMap.insert(std::make_pair("changeRate",findParam("parameters/parChangeRate", "Gain")));
    paramMap.insert(std::make_pair("leftMode",findParam("parameters/parLeftMode", "Gain")));
    paramMap.insert(std::make_pair("hitTime1",findParam("trajectory params/parTHit1", "Gain")));
    paramMap.insert(std::make_pair("hitTime2",findParam("trajectory params/parTHit2", "Gain")));
    paramMap.insert(std::make_pair("hitPos1",findParam("trajectory params/parP1Des", "Gain")));
    paramMap.insert(std::make_pair("hitPos2",findParam("trajectory params/parP2Des", "Gain")));
    paramMap.insert(std::make_pair("forceOn",findParam("Force On", "Value")));
    paramMap.insert(std::make_pair("trajMode",findParam("parameters/parTrajMode", "Gain")));
    paramMap.insert(std::make_pair("maxTrajDur",findParam("parameters/parMaxTrajDur", "Gain")));
    paramMap.insert(std::make_pair("fixedDur",findParam("parameters/parFixedDur", "Gain")));
    paramMap.insert(std::make_pair("marker",findParam("parameters/parMarker", "Gain")));    
    paramMap.insert(std::make_pair("forceTrigger",findParam("parameters/parForceTrigger", "Gain")));
    paramMap.insert(std::make_pair("wiggleAmp",findParam("parameters/parWiggleAmp", "Gain")));    
    paramMap.insert(std::make_pair("softStopPos",findParam("parameters/parPStop", "Gain")));
}

FingerBot::~FingerBot(void){
}
