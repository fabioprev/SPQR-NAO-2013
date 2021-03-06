#pragma once

#include "../Filters/MultiObjectParticleFilter.h"
#include <Manfield/manifoldprocessor.h>

namespace PTracking
{
	/**
	 * @class MultiTrackerProcessor
	 * 
	 * @brief Class that implements a single agent processor for the particle filter designed by Wu et al.
	 * 
	 * Reference paper: "Boosted Interactively Distributed Particle Filter for Automatic Multi-Object Tracking". In 15th IEEE International Conference on Image Processing, 2008.
	 */
	class MultiTrackerProcessor : public manfield::ManifoldFilterProcessor
	{
		protected:
			/**
			 * @brief value representing the maximum time (in milliseconds) to wait before executing a processing step.
			 */
			static const float MAX_TIME_TO_WAIT = 100.0;
			
			/**
			 * @brief last position of the robot.
			 */
			Point2of lastRobotPose;
			
			/**
			 * @brief timestamp of the last iteration.
			 */
			Timestamp timeOfLastIteration;
			
		public:
			/**
			 * @brief Empty constructor.
			 */
			MultiTrackerProcessor();
			
			/**
			 * @brief Destructor.
			 */
			~MultiTrackerProcessor();
			
			/**
			 * @brief Function that processes, if needed, the observations gathered from the sensors between the previous and current iteration.
			 * 
			 * @param robotPose reference to the position of the robot when the observations have been acquired.
			 * @param targetSeen true if a target has been seen, false otherwise.
			 * @param initialTimestamp reference to the timestamp of the previous iteration.
			 * @param currentTimestamp reference to the timestamp of the current iteration.
			 * @param readings reference to the observations gathered from the sensors between the previous and current iteration.
			 */
			void processReading(const Point2of& robotPose, bool targetSeen, const Timestamp& initialTimestamp, const Timestamp& currentTimestamp, vector<ObjectSensorReading>& readings);
			
			/**
			 * @brief Function that invokes the predict and update step of the underlying particle filter.
			 * 
			 * @param f reference to the underlying particle filter.
			 * @param targetSeen true if a target has been seen, false otherwise.
			 * @param initialTimestamp reference to the timestamp of the previous iteration.
			 * @param currentTimestamp reference to the timestamp of the current iteration.
			 * @param readings reference to the observations gathered from the sensors between the previous and current iteration.
			 */
			void singleFilterIteration(MultiObjectParticleFilter& f, bool targetSeen, const Timestamp& initialTimestamp, const Timestamp& currentTimestamp, vector<ObjectSensorReading>& readings) const;
			
			/**
			 * @brief Function that checks if the processing step is needed (i.e. a time of MAX_TIME_TO_WAIT is passed or the robot sufficiently moved either in translation or orientation).
			 * 
			 * @param robotPose reference to the current position of the robot.
			 * @param currentTimestamp reference to the timestamp of the current iteration.
			 * 
			 * @return \b true if the processing step is needed, \b false otherwise.
			 */
			bool updateNeeded(const Point2of& robotPose, const Timestamp& currentTimestamp) const;
			
			PARAM_SET_GET(float, updateFrequency, private, public, public)
			PARAM_SET_GET(unsigned int, nFusedParticles, private, public, public)
	};
}
