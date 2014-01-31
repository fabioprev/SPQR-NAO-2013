#pragma once

#include <string>

namespace SPQR
{
	/************ GAME CONTROLLER ************/
	static const std::string IP_GOALIE 									= "10.0.19.14";
	static const int CHEST_BUTTON_MANUAL_GAME_CONTROLLER_PORT				= 18003;
	static const int FIELD_DIMENSION_X										= 3000;
	static const int FIELD_DIMENSION_Y										= 2000;
	
	static const unsigned int POLICY										= 0;		///{STABLE ="0", S_POSITIONIG_X ="1", S_POSITIONIG_XY ="2", WALL ="3", TANK ="4", STATIC_POSITIONG="5"};
	static const unsigned int STRATEGY									= 0;		///{DRIBBLING ="0", PASSING ="1"};
	
	static const float TURN_VALID_THS										= 10;		/// degree
	static const float TURN_EXCESS										= 10;
	
	static const int COORDINATION_PORT_NUMBER								= 11937;
	static const int MAXIMUM_DISTANCE_BALL_VIEWED							= 6000;
	static const int MAXIMUM_DISTANCE_ON_THE_FIELD							= 11000;
    static const unsigned int TABLE_ROWS									= 5;		/// TABLE_ROWS also equals to the number of roles.
    static const unsigned int TABLE_COLUMNS								= 10;
	static const unsigned int DEAD_ROBOT_TIME_THRESHOLD					= 5000;
	static const unsigned int HYSTERESIS_PERIOD_IN_CYCLES					= 100;
	static const unsigned int COORDINATION_INFORMATION_NETWORK_FREQUENCY	= 10;		/// FREQUENCY!
	static const unsigned int FALL_DOWN_PENALTY							= 200;
	static const unsigned int TIME_TO_GET_UP								= 10000;
	static const unsigned int MOVING_BALL_MIN_VELOCITY					= 10;		/// [mm/s]
	static const unsigned int SUPPORTER_MIN_TIME_WHEN_LAST_SEEN			= 500;		/// [ms]
	static const unsigned int DEFENDER_MIN_TIME_WHEN_LAST_SEEN			= 500;		/// [ms]
	static const unsigned int JOLLY_MIN_TIME_WHEN_LAST_SEEN				= 500;		/// [ms]
	static const int MINIMUM_PASSING_DISTANCE								= 1000;		/// [mm]
	static const int HYSTERESIS_BOUND_DISTANCE								= 300;		/// [mm]
	
    static const float DEFENDER_KICKOFF_DEFAULT_POSITION_X				= -0.55 * FIELD_DIMENSION_X;
    static const float DEFENDER_KICKOFF_DEFAULT_POSITION_Y				= 0.13 * FIELD_DIMENSION_Y;
	static const float DEFENDER_NO_KICKOFF_DEFAULT_POSITION_X				= -0.55 * FIELD_DIMENSION_X;
    static const float DEFENDER_NO_KICKOFF_DEFAULT_POSITION_Y				= 0.13 * FIELD_DIMENSION_Y;
	
	static const float SUPPORTER_KICKOFF_DEFAULT_POSITION_X				= -0.27 * FIELD_DIMENSION_X;
    static const float SUPPORTER_KICKOFF_DEFAULT_POSITION_Y				= 0.33 * FIELD_DIMENSION_Y;    
	static const float SUPPORTER_NO_KICKOFF_DEFAULT_POSITION_X			= -0.27 * FIELD_DIMENSION_X;
    static const float SUPPORTER_NO_KICKOFF_DEFAULT_POSITION_Y			= 0.33 * FIELD_DIMENSION_Y;
	
	static const float JOLLY_KICKOFF_DEFAULT_POSITION_X					= -0.27 * FIELD_DIMENSION_X;
    static const float JOLLY_KICKOFF_DEFAULT_POSITION_Y					= -0.33 * FIELD_DIMENSION_Y;
	static const float JOLLY_NO_KICKOFF_DEFAULT_POSITION_X				= -0.27 * FIELD_DIMENSION_X;
    static const float JOLLY_NO_KICKOFF_DEFAULT_POSITION_Y				= -0.33 * FIELD_DIMENSION_Y;
	
	static const float STRIKER_KICKOFF_POSITION_X							= -220.0;
	static const float STRIKER_KICKOFF_POSITION_Y							= 0.0;
	static const float STRIKER_NO_KICKOFF_POSITION_X						= -1200.0;
	static const float STRIKER_NO_KICKOFF_POSITION_Y						= 0.0;
	
	static const float SPEED_X											= 0.6;
	static const float SPEED_Y											= 0.6;
	static const float HEAD_ROTATION										= 8.0;
	static const float TIME_BEFORE_STARTING_TO_COORD_SEARCH				= 7000.0;
	
	/************ WALL ************/
	static const float DEFENDER_KICKOFF_WALL_POSITION_X					= -0.75 * FIELD_DIMENSION_X;
    static const float DEFENDER_KICKOFF_WALL_POSITION_Y					= 0.16 * FIELD_DIMENSION_Y;
    static const float SUPPORTER_KICKOFF_WALL_POSITION_X					= -0.75 * FIELD_DIMENSION_X;
    static const float SUPPORTER_KICKOFF_WALL_POSITION_Y					= 0.45 * FIELD_DIMENSION_Y;
	static const float JOLLY_KICKOFF_WALL_POSITION_X						= -0.75 * FIELD_DIMENSION_X;
    static const float JOLLY_KICKOFF_WALL_POSITION_Y						= -0.30 * FIELD_DIMENSION_Y;
	
	/************ NO BALL ************/
	static const float DEFENDER_KICKOFF_NO_BALL_POSITION_X				= -0.75 * FIELD_DIMENSION_X;
    static const float DEFENDER_KICKOFF_NO_BALL_POSITION_Y				= 0.75 * FIELD_DIMENSION_Y;
    static const float SUPPORTER_KICKOFF_NO_BALL_POSITION_X				= -0.75 * FIELD_DIMENSION_X;
    static const float SUPPORTER_KICKOFF_NO_BALL_POSITION_Y				= -0.75 * FIELD_DIMENSION_Y;
	static const float JOLLY_KICKOFF_NO_BALL_POSITION_X					= 0.75 * FIELD_DIMENSION_X;
    static const float JOLLY_KICKOFF_NO_BALL_POSITION_Y					= -0.50 * FIELD_DIMENSION_Y;
    
	/************ GOALIE ************/
    static const float			GOALIE_BASE_POSITION_X					= -FIELD_DIMENSION_X + 250;	/// [mm]  //TODO take this from theFieldDimensions

	static const float			GOALIE_BASE_POSITION_Y					= 0;		/// [mm]
	static const float			GOALIE_BASE_POSITION_BEARING			= 0;		/// [mm]

	static const float			GOALIE_POSE_X_TOLLERANCE				= 150;		/// [mm]
	static const float			GOALIE_POSE_Y_TOLLERANCE				= 150;		/// [mm]
	static const float			GOALIE_POSE_ANGLE_TOLLERANCE			= 10;		/// [deg]
	static const float			GOALIE_POSE_X_TOLLERANCE_AFTER_DIVE		= 150;		/// [mm]
	static const float			GOALIE_POSE_Y_TOLLERANCE_AFTER_DIVE		= 150;		/// [mm]

	static const float			GOALIE_DIVE_TIME						= 3000;		/// [ms] //TODO set properly IMPORTANT: time for the robot to reach save position ( hands on ground ) since the dive command has been send
	static const float			GOALIE_MOVING_BALL_MIN_VELOCITY			= 10;		/// [mm/s]
	static const float			GOALIE_EPSILON_COLLINEAR				= 0.001;	/// [??]
	static const float			GOALIE_FAR_LIMIT_Y						= 800;		/// a little more than goal post   //TODO take this from FieldDimensions
	static const float			GOALIE_CLOSE_LIMIT_Y					= 200;		/// dont-dive distance  //TODO take this from FieldDimensions
	static const unsigned int	GOALIE_MIN_TIME_WHEN_LAST_SEEN				= 500;		/// [ms]
	static const float			GOALIE_MIN_BALL_DIST_FROM_POST			= 500;
	
	static const float			GOALIE_MAX_DIST_BALL_IN_RANGE_ABS		= 500;		/// [mm]
}
