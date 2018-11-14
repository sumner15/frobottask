///////////////////////////////////

#ifndef FingerTH
#define FingerTH

#define MAXLTH  320		// max number of running average elements

class TargetColor
{
public:
	int red;
	int green;
	int blue;
	int saturation;
} ;

class FingerT
{
private:
	int cursorsno;
	int targetno;
	int targetMode;
	int taskphase;
	int cursor_n;
	float ahigh;
	float alow;
	float auto_adjust;
	int aControl;
	int amin;
	int acount;
	float sum[MAXLTH];
	float asum;
	void update_target_color( float );	
	int prepos1;				// finger position at beginning of trial
	int prepos2;
	int minposchange;
	int maxposchange;
	float small_adj;
	float large_adj;
public:
	FingerT();
	void Config(int, int, int, float, float, int, int, float, float);
	void Update( int, int, int, int, int, int, int );
	void UpdateFingers(int, int);
	float getALow( void );
	float getAHigh( void );
	int GetTargetColor( int );
	int GetCursorColors();
	int RSVP(int, int);
	int accumulate( float );
	TargetColor target_color;
	float getPosof( int );
	float posof1;
	float posof2;
};

#endif