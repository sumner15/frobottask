#pragma once

#include <string>

class XPCModel
{
public:
	XPCModel(std::string, std::string, std::string);
	~XPCModel(void);	
	void	connectToTarget(std::string ipAddress, std::string ipPort);
	void	loadControllerModel(std::string mdlName);
	double	getParamById(int parIdx);
	double  getSignalById(int id);
	void	setParamById(int parIdx, double parValue);
	int		findParam(char *block, char *param);
	int		findSignal(char *sig);
	void	cleanUp(void);

private:
	void	checkError(char *str);	
	int		port;

};

