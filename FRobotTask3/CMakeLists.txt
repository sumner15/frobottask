###########################################################################
## $Id:  $
## Authors:
## Description: Build information for the FRobotTask application module
##              
##              
###########################################################################

# Set the executable name
SET( EXECUTABLE_NAME FRobotTask3 )

# Set the project specific sources
SET( SRC_PROJECT
  # MyNewCustomFilter.cpp    # Add new filters here to specify the module's behaviour   
  FRobot3.cpp 
  FRobotT3.cpp  
  newFingerBot.cpp
  stdafx.cpp
  xpcinitfree.c
  newXPCModel.cpp
)

SET( HDR_PROJECT
  # MyNewCustomFilter.h      # Don't forget the corresponding headers
  FRobot3.h  
  FRobotT3.h 
  newFingerBot.h
  stdafx.h
  newXPCModel.h
  targetver.h
  xpcapi.h
  xpcapiconst.h
)

# Use the BCI2000_INCLUDE macro if you need to link with frameworks from /src/extlib:
# BCI2000_INCLUDE( "3DAPI" )

# We're done. Add the application module to the Makefile or compiler project file:
BCI2000_ADD_APPLICATION_MODULE( 
  "${EXECUTABLE_NAME}" 
  "${SRC_PROJECT}" "${HDR_PROJECT}" 
)

