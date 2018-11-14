////////////////////////////////////////////////////////////////////////////////
// $Id: FRobot3.cpp 8/25/16
// Author: d mcfarland
// Description: A finger robot control task
//
// $BEGIN_BCI2000_LICENSE$
// 
// This file is part of BCI2000, a platform for real-time bio-signal research.
// [ Copyright (C) 2000-2012: BCI2000 team and many external contributors ]
// 
// BCI2000 is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
// 
// BCI2000 is distributed in the hope that it will be useful, but
//                         WITHOUT ANY WARRANTY
// - without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// $END_BCI2000_LICENSE$
////////////////////////////////////////////////////////////////////////////////

#include "FRobot3.h"
#include "FRobotT3.h"
#include "Color.h"
#include "Localization.h"

#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsSimpleTextItem>

RegisterFilter( FingerTask, 3 );

using namespace std;

FingerT fingert;

FingerTask::FingerTask()
: mRunCount( 0 ),
  mTrialCount( 0 ),
  mCursorPosX( 0.0 ),
  mCursorPosY( 0.0 ),
  mCursorSpeedX( 0.0 ),
  mCursorSpeedY( 0.0 ),
  mpWindow( NULL ),
  mpScene( NULL ),
  mpSceneView( NULL ),
  mpLabel( NULL ),
  mpScore( NULL ),			// djm added 2/18/16
  mpCurrent( NULL ),
  mpTarget( NULL ),
  mpCursor2( NULL ),		// djm added 11/17/14
  mpCursor( NULL )
{
  BEGIN_PARAMETER_DEFINITIONS
    "Application:Window int WindowWidth= 640 640 0 % "
      " // width of application window",
    "Application:Window int WindowHeight= 480 480 0 % "
      " // height of application window",
    "Application:Window int WindowLeft= 0 0 % % "
      " // screen coordinate of application window's left edge",
    "Application:Window int WindowTop= 0 0 % % "
      " // screen coordinate of application window's top edge",
      "Application:Cursor int CursorColor= 0xFF0000 0xFF0000 % % "
      " // feedback cursor color (color)",
    "Application:Cursor float CursorWidth= 5 5 0.0 % "
      " // feedback cursor width in percent of screen width",
	"Application:Cursor int CursorNumber= 2 2 1 3 "
      " // feedback cursor width in percent of screen width",
	 "Application:Cursor intlist CursorSequence= 0 1 % % "
	 "  // Fixed sequence of finger cues (empty if random) ",
	 "Application:Task int TaskPhase= 1 1 1 3 "
	  " // Phase of Finger Task ",
	"Application:Task int NullTargetMode= 1 1 0 1 "
	  " // if 1 then Blue target is NULL (not shown) ",
	 "Application:Task int MinAccTime= 1s 1s 0 % "
	 " // minimum time to accumulate EEG ",
	 "Application:Task float LowAdjust= 0.0040 0.0040 0.00 0.10 "
	 " // increment on miss ",
	 "Application:Task float HighAdjust= 0.0080 0.0080 0.00 0.10 "
	 " // decrement on hit ",
	"Application:Task float AccLow= -0.4 -0.4 -2.0 2.0 "
	 " // low value for transition ",
	"Application:Task float AccHigh= 0.4 0.4 -2.0 2.0 "
	 " // high value for transition ",
	"Application:Task int AccControl= 2 2 1 2 "
	 " // 1 Running Average of EEG  2 Cumulative EEG ",
	"Application:Task int Min4Move= 30 0 -100 100 "
	" // Minimum finger position change for a response ",
	"Application:Task int Max4NotMove= 15 0 -100 100 "
	" // Maximum finger position change for no response ",
	"Application:Task int DisplayScore= 1 0 0 1 "
	" // 1 to Display Score Statistics ",
	"Robot:Location string FRobotAdr= 169.254.201.253"
	   " // FRobot IP address",
	"Robot:Location string FRobotCOMport= 22222"
	   " // FRobot COM port",
	"Robot:Location string modelName= brainFINGER"
	   " // FingerRobot model name",
	"Robot:Assist int FRobotMove= 0 0 0 2 "
	    " // Assist?  0= none 1= yes no errors 2= yes allow errors ",
	"Robot:Assist float Kp= 2.0 2.0 0 2.0 "
	    " // PD movement proportional error gain ",
	"Robot:Assist float Kd= 0.2 0.2 0 0.2 "
		" // PD movement differential error gain ",
	"Robot:Assist float MovementDelay= 1.0 0 0 2.0 "
		" // Delay of movement (msec) after colored targets ",
	"Robot:Assist float MovementDuration= 1.0 0 0 2.0 "
		" // Total Possible duration (sec) of movement ",
	"Robot:Assist float MaxTrajDur= 1.7 0 0 5.0 "
		"// Maximum duration (sec) of movement ",
	"Robot:Assist float POSinitial= 0.0 0.9 0.0 1.0 "
	    " // starting and ending robot positions ",
	"Robot:Assist float POSmov2= 0.9 0.0 0.0 1.0 "
		" // position to move to ",
	"Robot:Assist float VThresh= 0.1 0.0 0.0 5.0 "
		" // velocity threshold for movement ",
	"Robot:Assist float FThresh= 0.1 0.0 0.0 5.0 "
		" // force threshold for movement ",
	"Robot:Assist float FRMode= 4.0 1.0 1.0 4.0 "
		" //  1 user initiated with return 2 autostart return 3 auto no return 4 user initiated no return ",
	"Robot:Positions int AutoReturn= 0 1 0 1 "
		" // 0 no supplemental autoreturn"    
		

  END_PARAMETER_DEFINITIONS

  BEGIN_STATE_DEFINITIONS		// djm added 11/19/14
	"CursorColors 4 0 0 0 ",
	"CursorResult 4 0 0 0 ",
	"ReactionTime 16 0 0 0 ",
	"TaskState3 8 0 0 0 ",	
	"FRobotPos1 16 0 0 0 ",
	"FRobotPos2 16 0 0 0 ",
//	"FRobotForceF1 16 0 0 0 ",
//	"FRobotForceF2 16 0 0 0 ", 	
	"FRobotForceF1a 16 0 0 0 ",
	"FRobotForceF2a 16 0 0 0 ", 
	"FRobotForceF1b 16 0 0 0 ",
	"FRobotForceF2b 16 0 0 0 ",
	"ITI 4 0 0 0"
  END_STATE_DEFINITIONS

  LANGUAGES "German",
  BEGIN_LOCALIZED_STRINGS
   "Timeout",          "Inaktiv",
   "Be prepared ...",  "Achtung ...",
  END_LOCALIZED_STRINGS

  mpWindow = new QWidget;
  mpWindow->setWindowFlags( Qt::FramelessWindowHint );
  mpWindow->setWindowTitle( "BCI2000 FINGERRobot prorgam" );

  mpScene = new QGraphicsScene;
  mpScene->setBackgroundBrush( QBrush( Qt::black ) );

  mpSceneView = new QGraphicsView( mpWindow );
  mpSceneView->setScene( mpScene );
  mpSceneView->setVerticalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
  mpSceneView->setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
  mpSceneView->show();

  mpLabel = new QGraphicsSimpleTextItem;
  mpScene->addItem( mpLabel );
  mpLabel->show();

  mpScore = new QGraphicsSimpleTextItem;		// djm added 11/17/14
  mpScene->addItem( mpScore );					
  mpScore->hide();

  mpCurrent = new QGraphicsSimpleTextItem;		// djm added 11/17/14
  mpScene->addItem( mpCurrent );					
  mpCurrent->hide();


  mpTarget = new QGraphicsRectItem;
  mpScene->addItem( mpTarget );
  mpTarget->setPen( Qt::NoPen );
  mpTarget->hide();

  mpCursor = new QGraphicsEllipseItem;
  mpCursor2 = new QGraphicsEllipseItem;		// djm added 11/19/14
  mpScene->addItem( mpCursor );
  mpScene->addItem( mpCursor2 );
  mpCursor->setPen( Qt::NoPen );
  mpCursor2->setPen( Qt::NoPen );
  mpCursor->hide();
  mpCursor2->hide();

  mpWindow->hide();

  srand( (unsigned)time(NULL) ) ;
}

FingerTask::~FingerTask()
{
  if( mpWindow )
    delete mpWindow;
  mpWindow = NULL;

  if( mpScene )
    delete mpScene;
  mpScene = NULL;
}

void FingerTask::UpdateStates( void )
{		
	unsigned short temp;
	
	temp= (unsigned short)(1000.0 * finger.getPos1());
	State("FRobotPos1")= temp;
	temp= (unsigned short)(1000.0 * finger.getPos2());
	State("FRobotPos2")= temp;

	temp= (unsigned short)(1000.0 * finger.getForceF1a());
	State("FRobotForceF1a")= temp;
	temp= (unsigned short)(1000.0 * finger.getForceF2a());
	State("FRobotForceF2a")= temp;	
	temp= (unsigned short)(1000.0 * finger.getForceF1b());
	State("FRobotForceF1b")= temp;
	temp= (unsigned short)(1000.0 * finger.getForceF2b());
	State("FRobotForceF2b")= temp;	

	/*
	
	temp= (unsigned short)(1000.0 * finger.getForceF1a());
	State("FRobotForceF1a")= temp;
	temp= (unsigned short)(1000.0 * finger.getForceF2a());
	State("FRobotForceF2a")= temp; 
	
	temp= 0.0; // (unsigned short)(1000.0 * finger.getForceF1b());
	State("FRobotForceF1b")= temp;
	temp=  0.0; // (unsigned short)(1000.0 * finger.getForceF2b());
	State("FRobotForceF2b")= temp;		

	*/
}


void FingerTask::Mov2( void )
{
	
	finger.setTrajMode( FRTrajmode );          //  mode 2 is for both flexes and extends  3 is just to desired position  4 is new
	currentTargetTime= finger.getTargetTime();
	double hitTime= currentTargetTime + maxtrajdur + MovementDelay;

	finger.setKp(Kp,2);		// robot PD controller gains to 0
	finger.setKd(Kd,2);	

	if( FRobotMove == 2 )
	{
		finger.setHitPos(mov2POS,2);
		finger.setHitTimes(hitTime,2);
	}
	else
	{
		switch( State("CursorColors")  )
		{
			case 1: finger.setHitPos(mov2POS,0);
					finger.setHitTimes( hitTime,0);
					break;
			case 2: finger.setHitPos(mov2POS,1);
					finger.setHitTimes(hitTime,1);
					break;
			case 3:	finger.setHitPos(mov2POS,2);
					finger.setHitTimes(hitTime,2);
					break;
			default:
				break;
		}
	}
}
void FingerTask::Return2( void )
{	
	finger.setKp(Kp,2);		// robot PD controller gains to 0
	finger.setKd(Kd,2);	
	
	
	finger.setTrajMode( 3 );						//  3 is just to desired position - return in this case
	currentTargetTime= finger.getTargetTime();
	double hitTime2= currentTargetTime + MovementDuration + 0.1;		// short wait

	finger.setHitPos(initialPOS,2);
	finger.setHitTimes( hitTime2,2);
}

void
	FingerTask::Relax( void )
{
	 finger.setKp(0,2);		// robot PD controller gains to 0
	 finger.setKd(0,2);	
}


void
FingerTask::OnPreflight( const SignalProperties& Input ) const
{
  Parameter( "WindowHeight" );
  Parameter( "WindowWidth" );
  Parameter( "WindowLeft" );
  Parameter( "WindowTop" );
  Parameter( "FRobotMove" );
  Parameter("Kp");
  Parameter("Kd");
  Parameter("MovementDuration");
  Parameter("MovementDelay");
  Parameter("POSinitial");
  Parameter("POSmov2");
  Parameter("FRMode");
  Parameter("AutoReturn");
  Parameter("DisplayScore");
  Parameter("VThresh");
  Parameter("FThresh");
  Parameter("MaxTrajDur");
  Parameter("FRobotCOMport");
  Parameter("FRobotAdr");
  Parameter("modelName");
  Parameter("NullTargetMode");
  
  State( "FRobotPos1" );
  State( "FRobotPos2" );

  if( RGBColor( Parameter( "CursorColor" ) ) == RGBColor( RGBColor::NullColor ) )
    bcierr << "Invalid RGB value in CursorColor" << endl;

  if( Parameter( "FeedbackDuration" ).InSampleBlocks() <= 0 )
    bcierr << "FeedbackDuration must be greater 0" << endl;

  if( Input.IsEmpty() )
    bcierr << "Requires at least one entry in control signal" << endl;

  if( Input.Channels() > 1 )
    bciout << "Will ignore additional channels in control signal" << endl;

  if( Parameter("FeedbackDuration") < Parameter("MaxTrajDur") )
	  bciout << "FeedbackDuration < MaxTrajDur" << endl;
  if( Parameter("MovementDuration") > Parameter("MaxTrajDur") )
	  bciout << "MovementDuration > MaxTrajDur" << endl;
  if( Parameter("ITIDuration") < Parameter("MovementDuration") )
	  bciout << "ITIDuration < MovementDuration" << endl; 
   if( Parameter("PreRunDuration") < 8 )
	  bciout << "PrerunDuration < 8s" << endl; 

		FRobotcomport= Parameter( "FRobotCOMport" );
		FRobotadr= Parameter( "FRobotAdr" );
		FRmodelName= Parameter( "modelName" );

		try
		{
			string ipAddress =  FRobotadr;            
			string ipPort = FRobotcomport;            
			string modelName = FRmodelName;           

			finger.config(ipAddress, ipPort, modelName);
			finger.cleanUp();		   
			Sleep(50);         // need to pause here
		}
		catch(...)
		{
			bcierr << "Error loading FingerBot" << endl;	
			return;
		}

	for(int i = 0; i < Parameter( "CursorSequence" )->NumValues(); i++ )
	{
		int val = Parameter( "CursorSequence" )( i );
		if( val < 0 || val > 3 )
		 bcierr << "CursorSequence contains illegal value " << val << endl;
	}  
}

void
FingerTask::OnInitialize( const SignalProperties& /*Input*/ )
{
  mpWindow->move( Parameter( "WindowLeft" ), Parameter( "WindowTop" ) );
  mpWindow->resize( Parameter( "WindowWidth" ), Parameter( "WindowHeight" ) );
  mpSceneView->resize( Parameter( "WindowWidth" ), Parameter( "WindowHeight" ) );
  mpSceneView->setSceneRect( 0, 0, Parameter( "WindowWidth"), Parameter( "WindowHeight" ) );
  mpSceneView->setVerticalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
  mpSceneView->setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
  mpSceneView->show();

  RGBColor textColor( RGBColor::Green );
  SetLabel( LocalizableString( "Timeout" ), textColor );
  RGBColor scoreColor( RGBColor::White );
  SetScore( LocalizableString( "Best Score = xyz" ), scoreColor );
  RGBColor currentColor( RGBColor::White );
  SetCurrent( LocalizableString( " Current = xyz" ), scoreColor );
  QRectF cursorRect;
  cursorRect.setWidth( mpWindow->width() * Parameter( "CursorWidth" ) / 100.0 );
  cursorRect.setHeight( cursorRect.width() );
  RGBColor cursorColor = RGBColor( Parameter( "CursorColor" ) );
  QBrush cursorBrush( QColor( cursorColor.R(), cursorColor.G(), cursorColor.B() ) );
  mpCursor->setRect( cursorRect );
  mpCursor2->setRect( cursorRect );
  mpCursor->setBrush( cursorBrush );
  mpCursor2->setBrush( cursorBrush );
  QRectF targetRect;
  targetRect.setLeft( (mpWindow->width() / 2) - (mpCursor->rect().width() ) );   
  targetRect.setWidth( mpCursor->rect().width() * 2 );
  if( Parameter( "NumberTargets" ) > 0 )
    targetRect.setHeight( mpCursor->rect().width() * 2 );  
  mpTarget->setRect( targetRect );

  
  mpWindow->show();
  State("CursorColors") = 0;				// djm 11/21/14
  State("ITI") = 0;
    
  CursorN= Parameter("CursorNumber");
  task_phase= Parameter("TaskPhase");
  amin= Parameter( "MinAccTime" ).InSampleBlocks();
  alow= Parameter( "AccLow" );
  ahigh= Parameter( "AccHigh" );
  aControl= Parameter( "AccControl" );  
  lowadj= Parameter("LowAdjust");
  highadj= Parameter("HighAdjust");
  minmov= Parameter("Min4Move");
  maxnot= Parameter("Max4NotMove");
  FRobotMove= Parameter("FRobotMove");
  Kp= Parameter("Kp");
  Kd= Parameter("Kd");
  MovementDuration= Parameter("MovementDuration");
  MovementDelay= Parameter("MovementDelay");
  initialPOS= Parameter("POSinitial");
  mov2POS= Parameter("POSmov2");
  vthresh= Parameter("VThresh");
  fthresh= Parameter("FThresh");
  maxtrajdur= Parameter("MaxTrajDur");
  FRobotcomport= Parameter( "FRobotCOMport" );
  FRobotadr= Parameter( "FRobotAdr" );
  FRmodelName= Parameter( "modelName" );  
  FRTrajmode= Parameter( "FRMode" );
  autoreturn= Parameter("AutoReturn");
  displayScore= Parameter("DisplayScore");
  targetMode= Parameter("NullTargetMode");
  currentScore= 0;
  maxScore= 0;
  if( mov2POS > initialPOS )
	  movFE = 1;
  else
	  movFE= 2;

  float rate= Parameter("ITIDuration").InSampleBlocks();
  float ucrate= Parameter("ITIDuration");
  UPDrate= rate/ucrate;  

  mFixedCursorSequence.clear();
  for(int i = 0; i < Parameter( "CursorSequence" )->NumValues(); i++ )			// djm 2/3/2017
    mFixedCursorSequence.push_back( Parameter( "CursorSequence" )( i ) );

}

void
FingerTask::OnStartRun()
{
  ++mRunCount;
  prerunflag= 0;
  mTrialCount = 0;
  mTrialStatistics.Reset();
  AppLog << "Run #" << mRunCount << " started" << endl;

  RGBColor textColor( RGBColor::Yellow );  

  try
	{
		string ipAddress =  FRobotadr;             
		string ipPort = FRobotcomport;             
		string modelName = FRmodelName;  
			
		finger.config(ipAddress, ipPort, modelName);

		fpos1= (int)(100.0 * finger.getPos1());	
		fpos2= (int)(100.0 * finger.getPos2());			

		AppLog   << "Run " << mRunCount << "   " << modelName << " Loaded: " << endl;
		Sleep(50);

		finger.setForcesOn(true);
		finger.setKp(Kp,2);
	    finger.setKd(Kd,2);
		finger.setTrajMode(FRTrajmode);
		finger.setMovementDuration(MovementDuration);
		finger.setVThresh(vthresh);
		finger.setFThresh(fthresh);	
		finger.setMaxTrajDur(maxtrajdur);
	}
	catch(...)
	{
		bcierr << "Error loading FingerBot" << endl;					
		return;
	}	
	 
  start1= (int)(100.0 * finger.getPos1());	
  start2= (int)(100.0 * finger.getPos2());	
  
   UpdateStates();

  startctr= 0;
  start_flag= 0;

  SetLabel( LocalizableString( "Relax" ), textColor );
  fingert.Config( State("TargetCode"), targetMode, Parameter("CursorNumber"), ahigh, alow, aControl, amin, lowadj, highadj );

  hits= 0;
  misses= 0;
  false_alarms= 0;
  correct_rejections= 0;
  iticount= 0; 
  
}

void
FingerTask::OnStopRun()
{
	AppLog   << "Run " << mRunCount << " finished: ";
	if( task_phase > 1 )
	{
		AppLog << mTrialStatistics.Total() << " trials, "
           << mTrialStatistics.Hits() << " hits, "
           << mTrialStatistics.Invalid() << " invalid.\n";
	
		int validTrials = mTrialStatistics.Total() - mTrialStatistics.Invalid();
		if( validTrials > 0 )
			AppLog << ( 200 * mTrialStatistics.Hits() + 1 ) / validTrials / 2  << "% correct \n";
	}
         
	if( task_phase != 2 )
	{
		AppLog << "Motor Task " << hits <<  " hits \n";
		AppLog << "           " << misses << " misses \n";
		AppLog << "           " << false_alarms << " false alarms \n";
		AppLog << "           " << correct_rejections << " correct rejections \n";
	}
  AppLog   << "====================="  << endl;

  RGBColor textColor( RGBColor::Green );
  SetLabel( LocalizableString( "Timeout" ), textColor );
  Parameter("AccLow")= fingert.getALow();
  Parameter("AccHigh")= fingert.getAHigh();

  finger.cleanUp();    		
}

void
FingerTask::OnTrialBegin()
{  
	ftime= 0;
	
  ++mTrialCount;
  AppLog.Screen << "Trial #" << mTrialCount
                << ", target: " << State( "TargetCode" )
                << endl;

  RGBColor textColor( RGBColor::Black );
  SetLabel( "", textColor );
  if( State( "TargetCode" ) > 0 )
  {
    QRectF targetRect = mpTarget->rect();
	QRectF newTargetRect( targetRect.left(),  ( mpWindow->height() / 2 ) - ( targetRect.height()/2 ) , targetRect.width(), targetRect.height() );   // djm  11/19/2014
	mpTarget->setRect( newTargetRect );	

    fingert.Update( State("TargetCode"), targetMode, task_phase, (int)(100.0 * finger.getPos1()), (int)(100.0 * finger.getPos2()), minmov, maxnot);	
	
	tred=        fingert.target_color.red;
	tgreen=      fingert.target_color.green;
	tblue=       fingert.target_color.blue;
	tsaturation= fingert.target_color.saturation;
	mpTarget->setBrush( QColor( tred, tgreen, tblue, tsaturation ) );		
  }

    UpdateCursorSize( 1.0 , 1.0 );
	mpCursor->hide();		  
	mpCursor2->hide(); 

	if( Parameter("CursorSequence")->NumValues() > 0 )
	{
		ccolors= mFixedCursorSequence[ ( mTrialCount - 1 ) % mFixedCursorSequence.size() ];
	}
	else
	{
		ccolors= fingert.GetCursorColors();
	}
  
  if( task_phase == 1 )
  {
	switch( ccolors )
	{
	case 0:
			mpCursor->setBrush( QBrush(Qt::blue) );
			mpCursor2->setBrush( QBrush(Qt::blue) );  
		break;
	case 1:
			mpCursor->setBrush( QBrush(Qt::yellow) );
			mpCursor2->setBrush( QBrush(Qt::blue) );  
		break;
	case 2:
			mpCursor->setBrush( QBrush(Qt::blue) );
			mpCursor2->setBrush( QBrush(Qt::yellow) );  
		break;
	case 3:
			mpCursor->setBrush( QBrush(Qt::yellow) );
			mpCursor2->setBrush( QBrush(Qt::yellow) );  
		break;
	}
  }
  else
  {
  
	switch( ccolors )
	{
	case 0:
			mpCursor->setBrush( QBrush(Qt::red) );
			mpCursor2->setBrush( QBrush(Qt::red) );  
		break;
	case 1:
			mpCursor->setBrush( QBrush(Qt::green) );
			mpCursor2->setBrush( QBrush(Qt::red) );  
		break;
	case 2:
			mpCursor->setBrush( QBrush(Qt::red) );
			mpCursor2->setBrush( QBrush(Qt::green) );  
		break;
	case 3:
			mpCursor->setBrush( QBrush(Qt::green) );
			mpCursor2->setBrush( QBrush(Qt::green) );  
		break;
	}	
  }

  mpCursor->update();
  mpCursor2->update();
  State("CursorColors") = ccolors;
  State("ReactionTime") = 0;  
 
  reaction_time= 0;
  pre_count= 0;
  pre_max= Parameter( "PreFeedbackDuration" ).InSampleBlocks();
  State("TaskState3") = 1;

  start1= (int)(100.0 * finger.getPos1()); 
  start2= (int)(100.0 * finger.getPos2());
  
 // State("FRobotPos1") = start1;
 // State("FRobotPos2") = start2;

  UpdateStates();

  if( task_phase == 1 )
  {
	  mpCursor->show();
	  mpCursor2->show();
  }
  else
  {
	  mpTarget->show();  
  }
  mpScore->hide();
  mpCurrent->hide();

  Relax();

  iticount= 0;
  itiPause= (int)(Parameter("MovementDuration") * UPDrate ); //  20; //  .InSampleBlocks();
  
}

void
FingerTask::OnTrialEnd()
{
  char maxscore[64];
  char currentscore[64];
 
  mpLabel->setText( QString( "" ) );
 
  RGBColor scoreColor( RGBColor::White );
  sprintf(maxscore,"Best Score =  %d",maxScore);
  SetScore( LocalizableString( maxscore ), scoreColor );

  RGBColor currentColor( RGBColor::White );
  sprintf(currentscore,"  Current  =  %d",currentScore);
  SetCurrent( LocalizableString( currentscore ), scoreColor );

  if( displayScore &&  (State("CursorResult") == 1)  )
  {
	mpScore->show();
	mpCurrent->show();
  }

  UpdateCursorSize( 1.0 , 1.0 );

  mpTarget->hide();
  mpCursor->hide();
  mpCursor2->hide();  

  if( State("CursorResult") == 1 )
	  hits++;
  else if(State("CursorColors") > 0 )
	  misses++;
  else if(State("CursorResult") > 0 )
	  false_alarms++;
  else 
	  correct_rejections++;

  State("CursorColors") = 0;
}

void
FingerTask::OnFeedbackBegin()
{  
    if( task_phase == 1 )
	{
		switch( ccolors )
		{
		case 0:
				mpCursor->setBrush( QBrush(Qt::red) );
				mpCursor2->setBrush( QBrush(Qt::red) );  
			break;
		case 1:
				mpCursor->setBrush( QBrush(Qt::green) );
				mpCursor2->setBrush( QBrush(Qt::red) );  
			break;
		case 2:
				mpCursor->setBrush( QBrush(Qt::red) );
				mpCursor2->setBrush( QBrush(Qt::green) );  
			break;
		case 3:
				mpCursor->setBrush( QBrush(Qt::green) );
				mpCursor2->setBrush( QBrush(Qt::green) );  
			break;
		}
		UpdateCursorSize( 1.0 , 1.0 );		
	}
	if( task_phase != 2 )
	{
	
		State("CursorResult") = 0;
		mpTarget->hide();
   	
		start1= (int)(100.0 * finger.getPos1()); 
		start2= (int)(100.0 * finger.getPos2());

		UpdateStates();
  
	//	State("FRobotPos1") = start1;
	//	State("FRobotPos2") = start2;

		if( FRobotMove > 0 )
		{									
			Mov2();
		}
	}	
}

void
FingerTask::OnFeedbackEnd()
{
  if( State( "ResultCode" ) == 0 )
  {
    AppLog.Screen << "-> aborted" << endl;
    mTrialStatistics.UpdateInvalid();
  }
  else
  {
    if( task_phase == 2 )
	{
		mTrialStatistics.Update( State( "TargetCode" ), State( "ResultCode" ) );
		if( State( "TargetCode" ) == State( "ResultCode" ) )
		{
			mpTarget->setBrush( QBrush( Qt::white ) );          //  djm 3/2/15 Changed from yellow to white
			mpTarget->update();
			AppLog.Screen << "-> hit" << endl;
		}
		else
		{
			AppLog.Screen << "-> miss" << endl;
		}
	}
	else
	{
		if( State("ResultCode") == 1 )
		{
			mTrialStatistics.Update( 1, 1 );	// they're both the same
			AppLog.Screen << "-> hit" << endl;
		}
		else
		{
			mTrialStatistics.Update( 2, 1 );	// they're different
			AppLog.Screen << "-> miss" << endl;
		}
	}	
  }  

  if( State("CursorResult") == 1 )
  {
	
	float reactionProp= (float)(reaction_time) / (float)(Parameter( "FeedbackDuration" ).InSampleBlocks());

    currentScore = (int)( 100.0 * ( 1.0 - reactionProp ) );

	if( currentScore > maxScore)
			maxScore= currentScore;
  }

  State("ReactionTime")= reaction_time;		
  reaction_time= 0; 
}


void
FingerTask::DoPreRun( const GenericSignal&, bool& doProgress )
{
	startctr++;

	if( prerunflag == 0 )
	{
		int tics= ( 3 * Parameter( "PreRunDuration" ).InSampleBlocks() ) /  4;
		if(startctr > tics)
		{
			Return2();
			prerunflag++;
		}
	}
	
	if( task_phase > 0 )                
	{
		doProgress= false;
		if( startctr > Parameter( "PreRunDuration" ).InSampleBlocks() )
			doProgress= true;
	}
	else
	{
		if( startctr < Parameter( "PreRunDuration" ).InSampleBlocks() )
		{
			doProgress= false;
			start1= (int)(100.0 * finger.getPos1());
			start2= (int)(100.0 * finger.getPos2());
		}
		else
		{
			
			if( start_flag== 0 )
			{
				RGBColor textColor( RGBColor::Green );
				SetLabel( LocalizableString( "Move to Start" ), textColor );
				doProgress= false;			
				if( ((int)(100.0 * finger.getPos1()) - start1) > minmov )
					start_flag= 1;
				if( ((int)(100.0 * finger.getPos2()) - start2) > minmov )
					start_flag= 1;
					
			}
			else
			{
				RGBColor textColor( RGBColor::Black );
				SetLabel( LocalizableString( "" ), textColor );
				if( start_flag ++ > Parameter( "PreRunDuration" ).InSampleBlocks() ) 
					doProgress= true;
				else
					doProgress= false;
			}
		}
	}
}

void
FingerTask::DoPreFeedback( const GenericSignal& ControlSignal, bool& doProgress )
{
	int result;
	State("ITI")= 0;	

	switch( task_phase )
	{
		case 1:
			fpos1= (int)(100.0 * finger.getPos1());	
			fpos2= (int)(100.0 * finger.getPos2());	
			fingert.UpdateFingers( fpos1, fpos2 );	

		//	State( "FRobotPos1" )= fpos1;
		//	State( "FRobotPos2" )= fpos2;

			UpdateStates();

			pre_count++;
			if( State("TaskState3") == 1 )			
			{
				int rnum= rand() % (pre_max - amin);				

				if( (rnum < 1) || ( pre_count > (pre_max - amin )  )  )  
				{
					State("TaskState3")= 2;
					pre_count= 0;
					fingert.UpdateFingers( fpos1, fpos2 );	
				}
				else if( pre_count > (pre_max - (amin + 1 )) )
				{
					State("TaskState3")= 2;
					pre_count= 0;
					fingert.UpdateFingers( fpos1, fpos2 );	
				}
			}
			else if( pre_count > amin )
			{
				fingert.UpdateFingers( fpos1, fpos2 );	
				State("TaskState3") = 3;				
				doProgress= true;						
			}
			break;
		case 2:			
			result= fingert.accumulate( ControlSignal( 0, 0 ) );

			State("ReactionTime")= result;

			tsaturation= fingert.target_color.saturation;
			mpTarget->setBrush( QColor( tred, tgreen, tblue, tsaturation ) );	

			if( result > 0 )
			{
				doProgress= true;

				if( result == State("TargetCode") )  
				{
					mpTarget->setBrush( QBrush(Qt::green) );			
				}
				State( "ResultCode" ) = result;		
			}  

			break;
		case 3:
			fpos1= (int)(100.0 * finger.getPos1());	
			fpos2= (int)(100.0 * finger.getPos2());	
			fingert.UpdateFingers( fpos1, fpos2 );	

			result= fingert.accumulate( ControlSignal( 0, 0 ) );
			
			if( result > 0 )
			{
				doProgress= true;					
					
				if( result == 3 )				// NULL Target transitions no matter what
					State("TaskState3")= 3;	

				else if(  result == State("TargetCode") )
					State("TaskState3")= 3;		
				else
					State("TaskState3")= 5;		// skip over Feedback
			}  

			break;
	}
}

void
FingerTask::DoFeedback( const GenericSignal&, bool& doProgress )
{		
	int res;

	res= 0;

	fpos1= (int)(100.0 * finger.getPos1());	
	fpos2= (int)(100.0 * finger.getPos2());	

	// State( "FRobotPos1" )= fpos1;
	// State( "FRobotPos2" )= fpos2;

	UpdateStates();

	State("ITI") = 0;
	
	if( task_phase == 2 )
	{
		if( State( "ResultCode" ) == State( "TargetCode" ) )
			mpTarget->setBrush( QBrush(Qt::green) );		
	}
	else
	{
		if( State("TaskState3") == 1 )
		{
			State("TaskState3") = 5;
			{
				doProgress= true;
			}
		}
		else if( State("TaskState3") == 5 )
		{
			doProgress= true;
		}
		else
		{
			if( ftime > 0 )
			{
					res= fingert.RSVP( fpos1, fpos2 );
					UpdateCursorSize( fingert.getPosof(1) , fingert.getPosof(2) );

					mpCursor->show();
					mpCursor2->show();
			}
			
			if( res  > 0 )
			{
				doProgress= true;		
				State("CursorResult") = res;
				State("TaskState3")= 4;
				if( res == 1 )
					State("ResultCode") = 1;
				else
					State("ResultCode") = 2;
			}
			reaction_time++;
		}
	}
	ftime++;
}

void
FingerTask::DoPostFeedback( const GenericSignal&, bool& doProgress )
{
		fpos1= (int)(100.0 * finger.getPos1());	
		fpos2= (int)(100.0 * finger.getPos2());	
		
	//	State( "FRobotPos1" )= fpos1;
	//	State( "FRobotPos2" )= fpos2;

		UpdateStates();
		
		if( State("CursorResult") == 1 )
		{
			switch( State("CursorColors") )
			{
			case 0:
				mpCursor->hide();
				mpCursor2->hide();
				break;
			case 1:
				mpCursor->setBrush( QBrush(Qt::white) );   // djm 3/2/15  changed from yellow
				mpCursor2->hide();
				break;
			case 2:
				mpCursor->hide();
				mpCursor2->setBrush( QBrush(Qt::white) );
				break;
			case 3:
				mpCursor->setBrush( QBrush(Qt::white) );
				mpCursor2->setBrush( QBrush(Qt::white) );
				break;
			}			
			mpCursor->update();
			mpCursor2->update();
			State("ResultCode") = State("TargetCode");  //  1;
		}
		else
		{
			mpCursor->hide();
			mpCursor2->hide();
			doProgress= true;
			State("ResultCode")= 0;
		}		
}

void
FingerTask::DoITI( const GenericSignal& , bool& /*doProgress*/ )
{
			
	fpos1= (int)(100.0 * finger.getPos1());	
	fpos2= (int)(100.0 * finger.getPos2());	

	// State( "FRobotPos1" )= fpos1;
	// State( "FRobotPos2" )= fpos2;

	UpdateStates();

	State( "TaskState3") = 0;
	State("ITI") = 1;	

	float thresh = ( mov2POS + initialPOS ) / 2.5;

	if( iticount > itiPause )
	{			
		if( autoreturn > 0 )
		{
			if( movFE == 1)
			{
				if(( finger.getPos1() > thresh) || (finger.getPos2() > thresh ))
					Return2();
			}
			else
			{
				if(( finger.getPos1() < thresh) || (finger.getPos2() < thresh ))
					Return2();
			}
		}	

		itiPause= 7734;   // large number we hopefully never reach
	}	
	iticount++;
}

void
FingerTask::SetLabel( const char *text, RGBColor &color )
{
  QFont labelFont;
  labelFont.fromString( "Arial" );
  labelFont.setPixelSize( mpWindow->height() / 8 );
  QFontMetrics fm( labelFont );
  mpLabel->setFont( labelFont );
  mpLabel->setPos( ( mpWindow->width() / 2 ) - ( fm.width( text ) / 2 ),
                   ( mpWindow->height() / 2 ) - ( fm.height() / 2 ) );
  mpLabel->setPen( QPen( QColor( color.R(), color.G(), color.B() ) ) );
  mpLabel->setBrush( QBrush( QColor( color.R(), color.G(), color.B() ) ) );
  mpLabel->setText( text );
}

void
FingerTask::SetScore( const char *text, RGBColor &color )
{
  QFont labelFont;
  labelFont.fromString( "Arial" );
  labelFont.setPixelSize( mpWindow->height() / 20 );
  QFontMetrics fm( labelFont );
  mpScore->setFont( labelFont );
  mpScore->setPos( ( mpWindow->width() / 12 ) ,
                   ( mpWindow->height() / 12 ) - ( fm.height() / 2 ) );
  mpScore->setPen( QPen( QColor( color.R(), color.G(), color.B() ) ) );
  mpScore->setBrush( QBrush( QColor( color.R(), color.G(), color.B() ) ) );
  mpScore->setText( text );
}
void
FingerTask::SetCurrent( const char *text, RGBColor &color )
{
  QFont labelFont;
  labelFont.fromString( "Arial" );
  labelFont.setPixelSize( mpWindow->height() / 20 );
  QFontMetrics fm( labelFont );
  mpCurrent->setFont( labelFont );
  mpCurrent->setPos( ( mpWindow->width() / 12 ) ,
                   ( mpWindow->height() / 12 ) + ( fm.height() ) );
  mpCurrent->setPen( QPen( QColor( color.R(), color.G(), color.B() ) ) );
  mpCurrent->setBrush( QBrush( QColor( color.R(), color.G(), color.B() ) ) );
  mpCurrent->setText( text );
}
void
FingerTask::UpdateCursorSize( float p1, float p2 )
{
	
	float cursorWidth= mpWindow->width() * p1 * Parameter( "CursorWidth" ) / 100.0;
	float cursorHeight= cursorWidth;	
	
	float cursor2Width= mpWindow->width() * p2 * Parameter( "CursorWidth" ) / 100.0;
	float cursor2Height= cursor2Width;

	float cursorPosY = mpWindow->height() / 4       - (mpCursor->rect().height()  / 2);				
	float cursor2PosY = 3 * mpWindow->height() / 4  - (mpCursor2->rect().height() / 2);

	float cursorPosX = ( (mpWindow->width() / 2) - (mpCursor->rect().width() / 2) );  
	float cursor2PosX = ( (mpWindow->width() / 2) - (mpCursor2->rect().width() / 2) );  

	QRectF cursorRect(cursorPosX, cursorPosY, cursorWidth,cursorHeight);
	QRectF cursorRect2(cursor2PosX, cursor2PosY, cursor2Width,cursor2Height);
  
	mpCursor->setRect( cursorRect );
	mpCursor2->setRect( cursorRect2 );
}  