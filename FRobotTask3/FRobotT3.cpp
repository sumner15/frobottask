//////////////////////////////////////

#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <Windows.h>

#include "FRobotT3.h"

FILE *tempf;


FingerT::FingerT()
{
	srand((unsigned)time(NULL));
}


void
FingerT::Config(int target, int mode, int n_cursors, float a_high, float a_low, int a_Control, int a_min, float smalla, float largea )
{
//	targetno= target;	
//	GetTargetColor( targetno );
	ahigh= a_high;
	alow= a_low;
	aControl= a_Control;
	amin= a_min;
	cursor_n= n_cursors;
	if( cursor_n < 3 )
			cursor_n= 3;	
    small_adj= smalla;
	large_adj= largea;

	acount= 0;

	targetMode= mode;
}

void FingerT::Update( int target, int mode, int phase, int r1, int r2, int min4, int max4not )
{
	acount= 0;
	targetMode= mode;
	taskphase= phase;
	targetno= target;
	GetTargetColor( targetno );
	prepos1= r1;
	prepos2= r2;
	minposchange= min4;
	maxposchange= max4not;	
}

void FingerT::UpdateFingers(int r1, int r2)
{
	prepos1= r1;
	prepos2= r2;
}

float FingerT::getALow( void )
{
	return(alow);
}
float FingerT::getAHigh( void )
{
	return(ahigh);
}


int
FingerT::GetTargetColor( int target_no )
{
	if( (targetMode == 1 ) && (taskphase == 3 ) && (targetno == 1) ) 
	{
		target_color.red= 0;
		target_color.green= 0;
		target_color.blue= 0;
		target_color.saturation= 0;
		return(1);
	}
	
	switch( target_no )
	{
	case 1:
		target_color.red= 127;
		target_color.green= 127;
		target_color.blue= 255;
		target_color.saturation= 127;
		break;
	case 2:
		target_color.red= 255;
		target_color.green= 255;
		target_color.blue= 0;
		target_color.saturation= 127;
		break;	
	default:
		return(-1);
	}
	return(1);
}

int
FingerT::GetCursorColors()
{
	int retval= rand() % (cursor_n + 1);    
	cursorsno= retval;

	return( retval);
}

float 
FingerT::getPosof( int fngr )
{
	if( fngr == 1 )
		return(posof1);
	else
		return(posof2);
}

int
FingerT::RSVP(int r1, int r2)
{
	int res;
	int finger1;
	int finger2;
	int pos1;
	int pos2;
	
	int sum;

	res= 0;
	finger1= 0;
	finger2= 0;

	pos1= r1 - prepos1;
	pos2= r2 - prepos2;
 

	// if( GetAsyncKeyState( 0x61 ) )         // used for keypad input
	//		finger1= 1;

	if( minposchange < 0 )  // extend
	{
		if( pos1 <= minposchange )
		{
			finger1= 1;
			if( pos2 < maxposchange )  // second finger past min for no response
				finger2= 2;
		}
		else if( pos2 <= minposchange )
		{
			finger2= 2;
			if( pos1 < maxposchange )  // first finger past min for no response
				finger1= 1;
		}
	}
	else   //  flex
	{	
		if( pos1 >= minposchange )
		{
			finger1= 1;
			if( pos2 > maxposchange )  // second finger past min for no response
				finger2= 2;
		}
		else if( pos2 >= minposchange )
		{
			finger2= 2;
			if( pos1 > maxposchange )  // first finger past min for no response
				finger1= 1;
		}
	}

	sum= finger1 + finger2;

	if( sum > 0 )
	{
		if( sum == cursorsno )
			res= 1;				// correct
		else 
			res= 2;				// error	
	}

	posof1= ((float)(pos1) / 2.0 + (float)(maxposchange)) / (float)(maxposchange);
	posof2= ((float)(pos2) / 2.0 + (float)(maxposchange)) / (float)(maxposchange);
			
	return(res);
}

////////////////////////////////////////////////////////////////////////////////////////////////
// This function adjusts target transparency based on proximity to the criterion
///////////////////////////////////////////////////////////////////////////////////////////////

void
FingerT::update_target_color( float rval )
{
	float delta;
	unsigned change;

	if( (targetMode == 1 ) && (taskphase == 3 ) && (targetno == 1) )
	{
		target_color.saturation= 0;
		return;
	}

	switch(targetno )
		{
			case 1:   // blue
				if( fabs(ahigh) > 0.0 )
					delta= (ahigh - rval) / ahigh; 				
				target_color.saturation= 255 - (int)(delta * 127.0);
				break;
			case 2:   // yellow
				if( fabs(alow) > 0.0 )
					delta= (alow - rval) / alow; 
				target_color.saturation= 255 - (int)(delta * 127.0);
				break;				
			default: change= 0; break;
		}
	
}

int
FingerT::accumulate( float sig )
{
	
	float result;
	int ret;
	int count;

	ret= 0;

	if( aControl == 1 )
		count= amin;
	else
		count= acount;

	for( int i=count;i>0;i-- )
			sum[i]= sum[i-1];

	sum[0]= sig;
	acount++;

	if( aControl == 2 )
			count++;

	if( acount >= amin )
	{
			if( (targetMode == 1) &&( targetno == 1) && (taskphase == 3 ) )		// NULL target for minimal accumulation						
			{
				return(3);
			}

			asum= 0;
			for(int i=0;i<amin;i++)
					asum+= sum[i];
			result= asum/count;

			update_target_color( result );

			if( result < alow )
			{
				ret= 2;
				alow-= large_adj;			// lower both
				ahigh -= small_adj;
			}
			else if( result > ahigh )
			{
				ret= 1;
				alow += small_adj;		// raise both
				ahigh += large_adj;
			}
			else
			{
				alow += small_adj / (acount + amin);		// both easier
				ahigh -= small_adj/ (acount + amin);
			}

//	FILE *tempfile;
//	tempfile= fopen("TempFile.txt","a+");
//	fprintf(tempfile,"ACount= %3d Ret= %2d   Result= %8.3f  alow= %8.5f  ahigh= %8.5f  \n",acount,ret,result,alow,ahigh);
//	fclose(tempfile);

	}
	return( ret );	
}