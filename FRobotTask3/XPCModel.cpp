#include "StdAfx.h"
#include "XPCModel.h"
#include <windows.h>
#include <iostream>

#include "xpcapiconst.h"
#include "xpcapi.h"

/**---------------------------- constructor -----------------------------------//
*/
XPCModel::XPCModel(std::string ipAddress, std::string ipPort, std::string mdlName) {
	if (xPCInitAPI()) {
        fprintf(stderr, "Could not load api\n");
        throw new std::exception("failed to load model");
    }
	std::cout << "port is " << port;
	connectToTarget(ipAddress, ipPort);
	loadControllerModel(mdlName);
	xPCStartApp(port);
}

/**------------------------- connect to the xpc target computer ---------------//
*/
void XPCModel::connectToTarget(std::string ipAddress, std::string ipPort) {
	int port_ = xPCOpenTcpIpPort(ipAddress.c_str() , ipPort.c_str());
	port = port_;
	//port = xPCOpenTcpIpPort(ipAddress.c_str() , ipPort.c_str());
	checkError("PortOpen: ");
}

/**------------------------ load the dlm file for the controller --------------//
* note: this assumes that the .mdl file is on the same path as the code.
*/
void XPCModel::loadControllerModel(std::string mdlName){
	xPCLoadApp(port, ".", mdlName.c_str()); 
	checkError("LoadApp: ");
    printf("Application %s loaded, SampleTime: %g  StopTime: %g\n\n", mdlName.c_str(), xPCGetSampleTime(port), xPCGetStopTime(port));
    checkError(NULL);
}

/**---------------------------- check for particular error --------------------//
* Checks for error by calling xPCGetLastError(); if an error is found, prints the 
* appropriate error message and exits.         
*/
void XPCModel::checkError(char *str) {
    char errMsg[80];
    if (xPCGetLastError()) {
        if (str != NULL)
            fputs(str, stderr);
        xPCErrorMsg(xPCGetLastError(), errMsg);
        fputs(errMsg, stderr);
        exit(EXIT_FAILURE);
    }
    return;
}

/**-------------------------------- find parameter id -------------------------//
* Wrapper function around the xPCGetParamIdx() API call. Also checks to see if 
* the parameter is not found, and exits in that case.                                                          
*/
int XPCModel::findParam(char *block, char *param) {
    int tmp;
    tmp = xPCGetParamIdx(port, block, param);
    if (xPCGetLastError() || tmp == -1) {
        fprintf(stderr, "Param %s/%s not found\n", block, param);
        exit(EXIT_FAILURE);
    }    
    return tmp;
} 

/**------------------------------ find signal id ------------------------------//
* Wrapper function around the xPCGetSignalIdx() API call. Also checks to see if 
* the signal is not found, and exits in that case.                                                          
*/
int XPCModel::findSignal(char *sig) {
    int tmp;
    tmp = xPCGetSignalIdx(port, sig);
    if (xPCGetLastError() || tmp == -1) {
        fprintf(stderr, "Signal %s not found\n", sig);
        exit(EXIT_FAILURE);
    }    
    return tmp;
}

/**------------------------- get a parameter from its id ----------------------//
* Wrapper function around the xPCGetParam() API call. Also checks for error, and 
* exits if an error is found.                     
*/
double XPCModel::getParamById(int paramIdx) {
    double p;
    xPCGetParam(port, paramIdx, &p);
    checkError("GetParam: ");
    return p;
} 

/**------------------------- get a signal from its id -------------------------//
* Wrapper function around the xPCGetSignal() API call.                     
*/
double XPCModel::getSignalById(int id){
	return xPCGetSignal(port, id);
}

/**----------------------------- set parameter --------------------------------//
* Wrapper function around the xPCSetParam() API call. Also checks
* for error, and exits if an error is found.                     
*/
void XPCModel::setParamById(int parIdx, double paramValue) {
    xPCSetParam(port, parIdx, &paramValue);
    checkError("SetParam: ");
    return;
}

/**------------------------- release all the things ---------------------------//
* Called at program termination to exit in a clean way.          
*/
void XPCModel::cleanUp(void) {
    xPCStopApp(port);
	xPCUnloadApp(port);
	xPCClosePort(port);
    xPCFreeAPI();
	port = 0;
    return;
}

/**--------------------------- destructor -------------------------------------//
*/
XPCModel::~XPCModel(void) {
	if (port){
		cleanUp();
	}
	
}
