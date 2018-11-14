////////////////////////////////////////////////////////////////////////////////
// $Id: FingerTask.h 4650 2013-11-22 16:31:14Z mellinger $
// Author: juergen.mellinger@uni-tuebingen.de
// Description: A demo feedback task.
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
#ifndef FINGER_TASK_H
#define FINGER_TASK_H

#include "FeedbackTask.h"
#include "TrialStatistics.h"
#include "newFingerBot.h"

using namespace std;

string FRobotcomport;
string FRobotadr;
string FRmodelName;

FingerBot finger;  

class FingerTask : public FeedbackTask
{
 public:
  FingerTask();
  virtual ~FingerTask();  
 private:
  // Events to be handled by FeedbackTask descendants.
  //  Events triggered by the GenericFilter interface
  virtual void OnPreflight( const SignalProperties& Input ) const;
  virtual void OnInitialize( const SignalProperties& Input );
  virtual void OnStartRun();
  virtual void OnStopRun();
  virtual void OnHalt()                                           {}
  //  Events triggered during the course of a trial
  virtual void OnTrialBegin();
  virtual void OnTrialEnd();
  virtual void OnFeedbackBegin();
  virtual void OnFeedbackEnd();

  //  Dispatching of the input signal.
  //  Each call to GenericSignal::Process() is dispatched to one of these
  //  events, depending on the phase in the sequence.
  //  There, each handler function corresponds to a phase.
  //  If a handler sets the "progress" argument to true, the application's
  //  state will switch to the next phase.
  virtual void DoPreRun(       const GenericSignal&, bool& doProgress );
  virtual void DoPreFeedback(  const GenericSignal&, bool& doProgress );
  virtual void DoFeedback(     const GenericSignal&, bool& doProgress );
  virtual void DoPostFeedback( const GenericSignal&, bool& doProgress );
  virtual void DoITI(          const GenericSignal&, bool& doProgress );

  void SetLabel( const char* text, RGBColor &color );
  void SetScore( const char* text, RGBColor &color );
  void SetCurrent( const char* text, RGBColor &color );
  void UpdateCursorSize( float, float);
  void UpdateStates( void );
  void Mov2( void );	
  void Return2( void );
  void Relax( void );


 private: 
	 std::vector<int> mFixedCursorSequence;
  int   mRunCount,
        mTrialCount;
  float mCursorPosX,
	    mCursor2PosX,
		mCursorPosY,
		mCursor2PosY,
        mCursorSpeedX,
        mCursorSpeedY;
  int   CursorN;
  int   reaction_time;
  int   task_phase;
  int	amin;
  int pre_count;
  int pre_max;
  float	alow;
  float	ahigh;
  float lowadj;
  float highadj;
  int   aControl;
  int tred;
  int tgreen;
  int tblue;
  int tsaturation;
  int minmov;
  int maxnot;
  int hits;
  int misses;
  int false_alarms;
  int correct_rejections;
  int start1,start2;
  int startctr;
  int start_flag;
  int fpos1;
  int fpos2;
  double currentTargetTime;
  int FRobotMove;
  float FRTrajmode;
  float initialPOS;
  float mov2POS;
  int movFE;
  float Kp;
  float Kd;
  float MovementDuration;
  float MovementDelay;
  float vthresh;
  float fthresh;
  float maxtrajdur;
  int ccolors;
  int ftime;
  int autoreturn;
  int displayScore;
  int currentScore;
  int maxScore;
  int iticount;
  int itiPause;
  int prerunflag;
  float UPDrate;
  int targetMode;
 
  TrialStatistics mTrialStatistics;

  class QWidget* mpWindow;
  class QGraphicsScene* mpScene;
  class QGraphicsView* mpSceneView;
  class QGraphicsSimpleTextItem* mpLabel;
  class QGraphicsSimpleTextItem* mpScore;
  class QGraphicsSimpleTextItem* mpCurrent;
  class QGraphicsRectItem* mpTarget;
  class QGraphicsEllipseItem* mpCursor2;		// djm added 11/17/14
  class QGraphicsEllipseItem* mpCursor;
};

#endif // FINGER_TASK_H
