#pragma once

#include "Tools/Math/Vector2.h"

class GlobalBallEstimation : public Streamable
{
	private:
		/** Streaming function
		* @param in  streaming in ...
		* @param out ... streaming out.
		*/
		void serialize(In* in, Out* out)
		{
			STREAM_REGISTER_BEGIN;
			STREAM(singleRobotX);
			STREAM(singleRobotY);
			STREAM(singleRobotVariance);
			STREAM(isSingleRobotValid);
			STREAM(multiRobotX);
			STREAM(multiRobotY);
			STREAM(multiRobotVariance);
			STREAM(isMultiRobotValid);
			STREAM_REGISTER_FINISH;
		}
		
	public:
		float singleRobotX;			/** The X position of the ball estimated by this robot on the field (global frame) */
		float singleRobotY;			/** The Y position of the ball estimated by this robot on the field (global frame) */
		float singleRobotVariance;	/** The variance of the ball estimation performed by this robot **/
		bool isSingleRobotValid;	/** True means that the estimation can be used, false otherwise **/
		
		float multiRobotX;			/** The X position of the ball estimated by all robots on the field (global frame) */
		float multiRobotY;			/** The Y position of the ball estimated by all robots on the field (global frame) */
		float multiRobotVariance;	/** The variance of the ball estimation performed by the whole robot **/
		bool isMultiRobotValid;		/** True means that the estimation can be used, false otherwise **/
		
		/** Constructor */
		GlobalBallEstimation() : singleRobotX(0.0), singleRobotY(0.0), singleRobotVariance(100.0), isSingleRobotValid(false),
								 multiRobotX(0.0), multiRobotY(0.0), multiRobotVariance(100.0), isMultiRobotValid(false) {;}
};
