#pragma once
#include "newxpcmodel.h"
#include <string>
#include <map>

class FingerBot : public XPCModel {
public:
	FingerBot();	
	~FingerBot(void);
	void config(std::string ipAddress, std::string ipPort, std::string mdlName);
	void setForcesOn(bool forceOn);
	void setHandMode(std::string handedness);
	void autoSetHandMode();
	void setMovementDuration(double dur);
	void setHitTimes(double hitTime,int fNum);
	void setHitPos(double pos,int fNum );
	void setTrajMode(double setting);
	void setVThresh(double setting);
	void setFThresh(double setting);
	void setMaxTrajDur(double setting);
	void zeroPropGains();
	void zeroDifGains();
	void setKp(double kp, int finger);
	void setKd(double kd, int finger);
	std::string checkGainInRange(double val, bool propGain);
	void setGainChangeRate(double rate);
	double FingerBot::getFeedback(int fNum);	
	void FingerBot::getPropGains(double& kp1, double& kp2);
	double FingerBot::getPos1();
	double FingerBot::getPos2();
	double FingerBot::getForce1();
	double FingerBot::getForce2();
	double FingerBot::getForceF1();
	double FingerBot::getForceF1a();
	double FingerBot::getForceF1b();
	double FingerBot::getForceF2();
	double FingerBot::getForceF2a();
	double FingerBot::getForceF2b();
	void setViscGains(double gain, int fNum);
	double FingerBot::getTargetTime();

private:
	bool rightHandMode;

	double	getParamByName(std::string paramName);
	void	setParamByName(std::string paramName, double paramValue);
	double	getSignalByName(std::string signalName);
	void	loadIdMap();	
	typedef std::map<std::string, int> IdMap; 
	IdMap	signalMap;
	IdMap	paramMap;

	struct UpcomingMove{
		int fNum;
		double time;
	};
	UpcomingMove upcomingMove; 

	static const double leftOrientAccel;  // the are probably machine dependant, so it is dumb for us to drop them here as if they are reliable constants - Just thought we should let you know that we know that.                                                                                                                                                                                                                                                             have a nice day.
	static const double rightOrientAccel;

	static const double KpRange[2]; 
    static const double KdRange[2];

};

