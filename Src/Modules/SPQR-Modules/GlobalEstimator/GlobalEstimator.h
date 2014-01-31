/**
* @file GlobalEstimator.h
*	This file declares a module that provides the global ball estimation using Distributed Data Fusion provided
*	by PTracking library (developed by Fabio Previtali).
* @author Fabio Previtali
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/SPQR-Representations/GlobalBallEstimation.h"
#include "Representations/Modeling/RobotPose.h"
#include <Core/Processors/Processor.h>
#include <Core/Processors/MultiAgentProcessor.h>
#include <Utils/AgentPacket.h>
#include <mutex>

MODULE(GlobalEstimator)
	REQUIRES(GameInfo)
	REQUIRES(RobotInfo)
	REQUIRES(RobotPose)
	REQUIRES(BallModel)
	REQUIRES(GroundTruthBallModel)
	REQUIRES(FrameInfo)
	PROVIDES_WITH_MODIFY(GlobalBallEstimation)
END_MODULE

class GlobalEstimator : public GlobalEstimatorBase
{
	private:
		static const unsigned int BALL_SEEN_THRESHOLD		= 500;
		static const int LAST_N_TARGET_PERCEPTIONS			= 1;
		
		std::map<int,std::pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f> > estimatedTargetModelsMultiAgent;
		std::map<int,std::pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f> > estimatedTargetModels;
		std::vector<PoseParticleVector> bestParticles;
		std::vector<std::pair<std::string,int> > receivers;
		std::vector<PTracking::ObjectSensorReadingMultiAgent> observationsMultiAgent;
		std::map<int,std::pair<int,std::pair<int,int> > > colorMap;
		PTracking::Processor processor;
		PTracking::MultiAgentProcessor multiAgentProcessor;
		PTracking::ObjectParticleFilter objectParticleFilter;
		PTracking::ObjectParticleFilterMultiAgent objectParticleFilterMultiAgent;
		PTracking::ObjectSensorReading::Observation targetVector[LAST_N_TARGET_PERCEPTIONS];
		PTracking::ObjectSensorReading objectSensorReading;
		PTracking::Point2of agentPose;
		PTracking::Timestamp currentTimestamp, initialTimestamp, initialTimestampMas, lastPrintTimeGlobal, lastPrintTimeLocal, lastTimeInformationSent;
		std::mutex mutex;
		std::string agentAddress, pViewerAddress;
		float falsePositiveThreshold, maxReading, messageFrequency, sizeMapX, sizeMapY, trueNegativeThreshold;
		int agentId, agentPort, bestParticlesNumber, counterResult, currentTargetIndex, iterationCounter, lastCurrentTargetIndex, lastTargetIndex, maxTargetIndex, pViewerPort;
		
		static void* waitAgentMessagesThread(GlobalEstimator* globalEstimator) { globalEstimator->waitAgentMessages(); return 0; }
		static void interruptCallback(int);
		
		std::string buildHeader() const;
		void configure(const std::string&);
		void estimatedBallConsensus(GlobalBallEstimation&);
		void init();
		std::string prepareDataForViewer() const;
		void sendEstimationsToAgents(const std::string& dataToSend) const;
		std::vector<PoseParticleVector> updateBestParticles(const std::map<int,std::pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f> >&);
		void updateTargetPosition(const std::map<int,pair<PTracking::ObjectSensorReading::Observation,PTracking::Point2f> >&);
		void updateTargetVector(PTracking::ObjectSensorReading&);
		void updateTargetVector(std::stringstream&);
		void waitAgentMessages();
		
	public:
		GlobalEstimator();
		
		void update(GlobalBallEstimation& globalBallEstimation);
};
