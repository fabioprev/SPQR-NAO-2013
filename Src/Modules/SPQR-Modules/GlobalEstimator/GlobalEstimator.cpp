/**
* @file GlobalEstimator.cpp
*	This file implements a module that provides the global ball estimation using Distributed Data Fusion provided
*	by PTracking library (developed by Fabio Previtali).
* @author Fabio Previtali
*/

#include "GlobalEstimator.h"
#include <Platform/SystemCall.h>
#include <Representations/SPQR-Representations/ConfigurationParameters.h>
#include <Core/Sensors/BasicSensor.h>
#include <UdpSocket.h>
#include <Manfield/configfile/configfile.h>
#include <sys/stat.h>
#include <signal.h>
#include <string.h>
#include <algorithm>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <unistd.h>
#include <vector>

// Uncomment to enable debug prints.
#define DEBUG_MODE ;

using namespace std;
using namespace PTracking;
using GMapping::ConfigFile;

GlobalEstimator::GlobalEstimator() : agentId(-1)
{
	pthread_t waitAgentMessagesThreadId;
	string configDirectory;
	char currentWorkingDirectory[1024];
	int counter;
	
	configDirectory = "";
	
	if (SystemCall::getMode() == SystemCall::simulatedRobot)
	{
		if (getcwd(currentWorkingDirectory,1024)) {;}
		
		configDirectory = currentWorkingDirectory;
		
		configDirectory = configDirectory.substr(0,configDirectory.rfind("/")) + "/";
	}
	else configDirectory = "Config/";
	
	initialTimestamp.setToNow();
	initialTimestampMas.setToNow();
	currentTimestamp.setToNow();
	counterResult = 0;
	currentTargetIndex = 0;
	iterationCounter = 0;
	lastCurrentTargetIndex = 0;
	lastTargetIndex = -1;
	maxTargetIndex = 0;
	
	processor.addSensorFilter(&objectParticleFilter);
	
	objectParticleFilter.configure(configDirectory + string("PTracking/parameters.cfg"));
	objectParticleFilter.initFromUniform();
	
	processor.init();
	
	multiAgentProcessor.addSensorFilter(&objectParticleFilterMultiAgent);
	
	objectParticleFilterMultiAgent.configure(configDirectory + string("PTracking/parameters.cfg"));
	
	objectParticleFilterMultiAgent.initFromUniform();
	
	multiAgentProcessor.init();
	
	objectSensorReading.setSensor(objectParticleFilter.getSensor());
	
	srand(time(0));
	
	counter = 1;
	
	for (int i = 0; i < 256; i += 63)
	{
		for (int j = 0; j < 256; j += 63)
		{
			for (int k = 0; k < 256; k += 63, ++counter)
			{
				colorMap.insert(make_pair(counter,make_pair(i,make_pair(j,k))));
			}
		}
	}
	
	WARN("Particles' number (PF Single-Agent): " << objectParticleFilter.getparticleNumber() << endl);
	WARN("Particles' number (PF Multi-Agent): " << objectParticleFilter.getparticleNumber() << endl);
	
	configure(configDirectory);
	
	pthread_create(&waitAgentMessagesThreadId,0,(void*(*)(void*)) waitAgentMessagesThread,this);
}

string GlobalEstimator::buildHeader() const
{
	stringstream header;
	int differentTypes;
	
	differentTypes = estimatedTargetModelsMultiAgent.size();
	
	if (abs(currentTargetIndex - lastCurrentTargetIndex) > 0) ++differentTypes;
	
	differentTypes += objectParticleFilter.getObservationsMapping().size();
	
	header << agentId << " " << differentTypes << " ";
	
	/// Pay attention: "true" or "false" means respectively whether the point is oriented or not. In the first case you must use Point2ofOnMap.
	//header << "0" << " " << differentType << " AgentPose true #0000ff " << 7 << " " << 2.0 << " ";
	
	for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimatedTargetModelsMultiAgent.begin(); it != estimatedTargetModelsMultiAgent.end(); ++it)
	{
		stringstream s;
		
		const map<int,pair<int,pair<int,int> > >::const_iterator& colorTrack = colorMap.find(it->first);
		
		s << "#" << setw(2) << setfill('0') << std::hex << colorTrack->second.second.second
				 << setw(2) << setfill('0') << std::hex << colorTrack->second.second.first
				 << setw(2) << setfill('0') << std::hex << colorTrack->second.first;
		
		header << "EstimatedTargetModelsWithIdentityMultiAgent false " << s.str() << " " << 6 << " " << (1.5 * agentId) << " ";
	}
	
	if (abs(currentTargetIndex - lastCurrentTargetIndex) > 0)
	{
		header << "TargetPerceptions false #0000ff " << 14 << " " << 2 << " ";
	}
	
	const vector<pair<Point2of,Point2of> >& observationsMapping = objectParticleFilter.getObservationsMapping();
	
	for (vector<pair<Point2of,Point2of> >::const_iterator it = observationsMapping.begin(); it != observationsMapping.end(); ++it)
	{
		header << "ObservationsMapping false #000000 " << 0 << " " <<  2 << " ";
	}
	
	return header.str();
}

void GlobalEstimator::configure(const string& configDirectory)
{
	vector<int> agentVector;
	ConfigFile fCfg;
	stringstream s;
	string agents, key, section;
	float worldXMax, worldXMin, worldYMax, worldYMin;
	bool isPresent;
	
	if (!fCfg.read(configDirectory + string("PTracking/agent.cfg")))
	{
		ERR("Error reading file '" << configDirectory + "PTracking/agent.cfg'. Exiting..."<< endl);
		
		exit(-1);
	}
	
	try
	{
		section = "parameters";
		
		/// It could have been set by using the constructor.
		if (agentId == -1)
		{
			key = "agentId";
			agentId = fCfg.value(section,key);
		}
		
		key = "agents";
		agents = string(fCfg.value(section,key));
		
		s << agents;
		
		while (s.good())
		{
			string temp;
			
			if (s.eof()) break;
			
			getline(s,temp,',');
			
			agentVector.push_back(atoi(temp.c_str()));
		}
		
		key = "messageFrequency";
		messageFrequency = fCfg.value(section,key);
		
		if (SystemCall::getMode() == SystemCall::simulatedRobot) section = "LocalAgent";
		else section = "Agent";
		
		const vector<string>& agentNames = Utils::getAgentsName(configDirectory + string("PTracking/agent.cfg"),string("[") + section + string("]"));
		
		int counter = 1;
		
		isPresent = false;
		
		for (vector<string>::const_iterator it = agentNames.begin(); it != agentNames.end(); it++)
		{
			if (find(agentVector.begin(),agentVector.end(),counter) == agentVector.end()) continue;
			
			key = (*it) + "Address";
			const string address = fCfg.value(section,key);
			
			key = (*it) + "Port";
			int p = fCfg.value(section,key);
			
			/// Checking if the agentId is present in the receivers' list. If so, the information between the local and global layer are exchanged by using the main memory.
			if (counter++ == agentId)
			{
				isPresent = true;
				agentAddress = address;
				agentPort = p;
				
				continue;
			}
			
			WARN("Adding receiver: " << address << ":" << p << endl);
			
			receivers.push_back(make_pair(address,p));
		}
		
		if (!isPresent)
		{
			ERR("The agent id " << agentId << " is not present in the list of the receivers... Please check the configuration! Exiting..." << endl);
			
			exit(-1);
		}
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	if (!fCfg.read(configDirectory + string("PTracking/parameters.cfg")))
	{
		ERR("Error reading file '" << configDirectory + "PTracking/parameters.cfg'. Exiting..."<< endl);
		
		exit(-1);
	}
	
	try
	{
		section = "parameters";
		
		key = "bestParticles";
		bestParticlesNumber = fCfg.value(section,key);
		
		key = "falsePositiveThreshold";
		falsePositiveThreshold = fCfg.value(section,key);
		
		key = "trueNegativeThreshold";
		trueNegativeThreshold = fCfg.value(section,key);
		
		section = "sensor";
		
		key = "maxReading";
		maxReading = fCfg.value(section,key);
		
		section = "location";
		
		key = "worldXMin";
		worldXMin = fCfg.value(section,key);
		
		key = "worldXMax";
		worldXMax = fCfg.value(section,key);
		
		key = "worldYMin";
		worldYMin = fCfg.value(section,key);
		
		key = "worldYMax";
		worldYMax = fCfg.value(section,key);
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
	
	sizeMapX = worldXMax - worldXMin;
	sizeMapY = worldYMax - worldYMin;
	
	if (!fCfg.read(configDirectory + string("PTracking/pviewer.cfg")))
	{
		ERR("Error reading file '" << configDirectory + "PTracking/pviewer.cfg'. Exiting..."<< endl);
		
		exit(-1);
	}
	
	try
	{
		section = "PViewer";
		
		key = "address";
		pViewerAddress = string(fCfg.value(section,key));
		
		key = "port";
		pViewerPort = fCfg.value(section,key);
	}
	catch (...)
	{
		ERR("Not existing value '" << section << "/" << key << "'. Exiting..." << endl);
		
		exit(-1);
	}
}

/**
 * WARNING: This function must be updated because it does not consider the target identity provided by PTracking.
 */
void GlobalEstimator::estimatedBallConsensus(GlobalBallEstimation& globalBallEstimation)
{
	vector<Point2f> allEstimations;
	multimap<int,int> consensus;
	int index;
	
	index = -1;
	
	if (estimatedTargetModelsMultiAgent.size() > 1)
	{
		for (vector<ObjectSensorReadingMultiAgent>::const_iterator it = observationsMultiAgent.begin(); it != observationsMultiAgent.end(); ++it)
		{
			const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimation = it->getEstimationsWithModels();
			
			for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it2 = estimation.begin(); it2 != estimation.end(); ++it2)
			{
				allEstimations.push_back(it2->second.first.observation.getCartesian());
			}
		}
		
		int counter;
		
		for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimatedTargetModelsMultiAgent.begin(); it != estimatedTargetModelsMultiAgent.end(); ++it)
		{
			counter = 0;
			
			for (vector<Point2f>::const_iterator it2 = allEstimations.begin(); it2 != allEstimations.end(); ++it2)
			{
				if (Utils::isTargetNear(it->second.first.observation.getCartesian(),*it2,1.0)) counter++;
			}
			
			consensus.insert(make_pair(counter,it->first));
		}
		
		index = consensus.rbegin()->second;
		
		if (consensus.count(consensus.rbegin()->first) != 1) return;
	}

	if (estimatedTargetModelsMultiAgent.size() > 0)
	{
		if (index == -1)
		{
			index = estimatedTargetModelsMultiAgent.begin()->first;
		}
		
		globalBallEstimation.multiRobotX = estimatedTargetModelsMultiAgent.at(index).first.observation.getCartesian().x * 1000.0;
		globalBallEstimation.multiRobotY = estimatedTargetModelsMultiAgent.at(index).first.observation.getCartesian().y * 1000.0;
		globalBallEstimation.multiRobotVariance = Utils::roundN(estimatedTargetModelsMultiAgent.at(index).second.mod(),2);
		globalBallEstimation.isMultiRobotValid = (globalBallEstimation.multiRobotVariance < 1.5) ? true : false;
	}
	else
	{
		globalBallEstimation.multiRobotVariance = 100.0;
		globalBallEstimation.isMultiRobotValid = false;
	}
}

string GlobalEstimator::prepareDataForViewer() const
{
	stringstream streamDataToSend;
	int i;
	
	streamDataToSend << buildHeader();
	
	//streamDataToSend << "1 " << Utils::Point2ofOnMap << " " << Utils::roundN(agentPose.x,2) << " " << Utils::roundN(agentPose.y,2) << " " << Utils::roundN(agentPose.theta,2);
	
	for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimatedTargetModelsMultiAgent.begin(); it != estimatedTargetModelsMultiAgent.end(); ++it)
	{
		streamDataToSend << " 1 " << Utils::Point2fOnMap << " " << Utils::roundN(it->second.first.observation.getCartesian().x,2) << " " << Utils::roundN(it->second.first.observation.getCartesian().y,2);
	}
	
	if (abs(currentTargetIndex - lastCurrentTargetIndex) > 0)
	{
		streamDataToSend << " " << ((lastCurrentTargetIndex > currentTargetIndex) ? ((LAST_N_TARGET_PERCEPTIONS - lastCurrentTargetIndex) + currentTargetIndex)
																				  : (currentTargetIndex - lastCurrentTargetIndex));
	}
	
	i = lastCurrentTargetIndex;
	
	while (i != currentTargetIndex)
	{
		streamDataToSend << " " << Utils::Point2fOnMap << " " << Utils::roundN(targetVector[i].observation.getCartesian().x,2) << " " << Utils::roundN(targetVector[i].observation.getCartesian().y,2);
		
		++i;
		
		if (i == LAST_N_TARGET_PERCEPTIONS) i = 0;
	}
	
	const vector<pair<Point2of,Point2of> >& observationsMapping = objectParticleFilter.getObservationsMapping();
	
	for (vector<pair<Point2of,Point2of> >::const_iterator it = observationsMapping.begin(); it != observationsMapping.end(); ++it)
	{
		streamDataToSend << " 1 " << Utils::Line2dOnMap << " " << Utils::roundN(it->first.x,2) << " " << Utils::roundN(it->first.y,2)
						 << " " << Utils::roundN(it->second.x,2) << " " << Utils::roundN(it->second.y,2);
	}
	
	return streamDataToSend.str();
}

void GlobalEstimator::sendEstimationsToAgents(const string& dataToSend) const
{
	UdpSocket senderSocket;
	int ret;
	
	for (vector<pair<string,int> >::const_iterator it = receivers.begin(); it != receivers.end(); it++)
	{
		ret = senderSocket.send(dataToSend,InetAddress(it->first,it->second));
		
		if (ret == -1)
		{
			ERR("Error when sending message to: '" << it->second << "'." << endl);
		}
	}
}

void GlobalEstimator::update(GlobalBallEstimation& globalBallEstimation)
{
	static UdpSocket senderSocket;
	
	vector<ObjectSensorReading> observations;
	Point2of robotPose;
	string dataToSend;
	double rho;
	bool ballSeen, isPrintTimeLocal;
	
	currentTimestamp.setToNow();
	
	isPrintTimeLocal = ((currentTimestamp - lastPrintTimeLocal).getMs() > 1000.0);
	
	if (isPrintTimeLocal) lastPrintTimeLocal.setToNow();
	
	robotPose.x = theRobotPose.translation.x / 1000.0;
	robotPose.y = theRobotPose.translation.y / 1000.0;
	robotPose.theta = theRobotPose.rotation;
	
#ifdef DEBUG_MODE
	if (isPrintTimeLocal)
	{
		cerr << "\033[22;31;1m[PTracking] RobotPose (" << theRobotInfo.number << ") ->: [" << Utils::roundN(robotPose.x,2) << "," << Utils::roundN(robotPose.y,2)
			 << "," << Utils::roundN(Utils::rad2deg(robotPose.theta),2) << "], validity: " << theRobotPose.validity << "\033[0m" << endl;
	}
#endif
	
	static int actualObservations = 1;
	
	// The robot is sufficiently well-localized.
	if (theRobotPose.validity > 0.6)
	{
		ballSeen = ((theFrameInfo.time > 500) && ((theFrameInfo.time - theBallModel.timeWhenLastSeen) < BALL_SEEN_THRESHOLD));
		
		if (ballSeen)
		{
			// Considering just the last ball observation (converting it in meters).
			targetVector[0].observation.rho = sqrt(((theBallModel.estimate.position.x / 1000.0) * (theBallModel.estimate.position.x / 1000.0)) + 
												   ((theBallModel.estimate.position.y / 1000.0) * (theBallModel.estimate.position.y / 1000.0)));
			
			targetVector[0].observation.theta = atan2(theBallModel.estimate.position.y / 1000.0,theBallModel.estimate.position.x / 1000.0);
			
			rho = sqrt((targetVector[0].observation.getCartesian().x * targetVector[0].observation.getCartesian().x) +
					   (targetVector[0].observation.getCartesian().y * targetVector[0].observation.getCartesian().y));
			
			actualObservations = 1;
			
#ifdef DEBUG_MODE
			if (isPrintTimeLocal)
			{
				cerr << "\033[22;32;1m[PTracking] Observation (" << theRobotInfo.number << ") ->: [" << Utils::roundN(targetVector[0].observation.getCartesian().x,2)
					 << "," << Utils::roundN(targetVector[0].observation.getCartesian().y,2) << "]\033[0m" << endl;
			}
#endif
			
			for (int i = 0; i < actualObservations; ++i)
			{
				if ((fabs(targetVector[i].observation.getCartesian().x) > sizeMapX) || (fabs(targetVector[i].observation.getCartesian().y) > sizeMapY)) ballSeen = false;
			}
		}
		
		objectSensorReading.setObservationsAgentPose(robotPose);
		objectSensorReading.setObservations(targetVector,ballSeen ? actualObservations : 0,0,LAST_N_TARGET_PERCEPTIONS,maxReading);
		
		observations.push_back(objectSensorReading);
		
		processor.processReading(robotPose,ballSeen,initialTimestamp,currentTimestamp,observations);
		
		const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimationsWithModel = objectParticleFilter.getEstimationsWithModel();
		
		updateTargetPosition(estimationsWithModel);
		bestParticles = updateBestParticles(estimationsWithModel);
		
		Point2f minVariance(100.0,100.0);
		
		for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimationsWithModel.begin(); it != estimationsWithModel.end(); ++it)
		{
			if (it->second.second.mod() < minVariance.mod())
			{
				globalBallEstimation.singleRobotX = it->second.first.observation.getCartesian().x * 1000.0;
				globalBallEstimation.singleRobotY = it->second.first.observation.getCartesian().y * 1000.0;
				globalBallEstimation.singleRobotVariance = Utils::roundN(it->second.second.mod(),2);
				globalBallEstimation.isSingleRobotValid = (it->second.second.mod() < 1.5) ? true : false;
				
				minVariance = it->second.second;
			}
		}
		
#ifdef DEBUG_MODE
		if (isPrintTimeLocal)
		{
			cerr << "\033[22;34;1m[PTracking] SingleRobot (" << theRobotInfo.number << ") -> [" << Utils::roundN(globalBallEstimation.singleRobotX / 1000.0,2)
				 << "," << Utils::roundN(globalBallEstimation.singleRobotY / 1000.0 ,2) << "], validity = " << globalBallEstimation.isSingleRobotValid
				 << ", variance: " << globalBallEstimation.singleRobotVariance << "\033[0m" << endl;
		}
#endif
		
		if (estimatedTargetModels.size() > 0)
		{
			dataToSend = "Agent ";
			
			AgentPacket agentPacket;
			
			agentPacket.dataPacket.ip = agentAddress;
			agentPacket.dataPacket.port = agentPort;
			agentPacket.dataPacket.agentPose = agentPose;
			agentPacket.dataPacket.estimatedTargetModels = estimatedTargetModels;
			agentPacket.dataPacket.particlesTimestamp = currentTimestamp.getMsFromMidnight();
			
			dataToSend += agentPacket.toString();
			
			if ((Timestamp() - lastTimeInformationSent).getMs() > (1000.0 / messageFrequency))
			{
				sendEstimationsToAgents(dataToSend);
				
				lastTimeInformationSent.setToNow();
			}
			
			ObjectSensorReadingMultiAgent objectSensorReadingMultiAgent;
			
			objectSensorReadingMultiAgent.setSensor(objectParticleFilter.getSensor());
			objectSensorReadingMultiAgent.setEstimationsWithModels(estimatedTargetModels);
			objectSensorReadingMultiAgent.setEstimationsTimestamp(currentTimestamp.getMsFromMidnight());
			
			mutex.lock();
			
			observationsMultiAgent.push_back(objectSensorReadingMultiAgent);
			
			mutex.unlock();
		}
		else
		{
			globalBallEstimation.singleRobotVariance = 100.0;
			globalBallEstimation.isSingleRobotValid = false;
		}
		
		initialTimestamp = currentTimestamp;
	}
	
	if ((currentTimestamp - initialTimestampMas).getMs() > (2000.0 / SPQR::COORDINATION_INFORMATION_NETWORK_FREQUENCY))
	{
		int ret;
		bool isPrintTimeGlobal;
		
		isPrintTimeGlobal = ((currentTimestamp - lastPrintTimeGlobal).getMs() > 1000.0);
		
		if (isPrintTimeGlobal) lastPrintTimeGlobal.setToNow();
		
		initialTimestampMas = currentTimestamp;
		
		mutex.lock();
		
		if (observationsMultiAgent.size() > 0)
		{
			if (observationsMultiAgent.size() > (SPQR::COORDINATION_INFORMATION_NETWORK_FREQUENCY * 5))
			{
				observationsMultiAgent.erase(observationsMultiAgent.begin(),observationsMultiAgent.end() - (SPQR::COORDINATION_INFORMATION_NETWORK_FREQUENCY * 5));
			}
			
			multiAgentProcessor.processReading(observationsMultiAgent);
			estimatedTargetModelsMultiAgent = objectParticleFilterMultiAgent.getEstimationsWithModel();
			estimatedBallConsensus(globalBallEstimation);
			
			observationsMultiAgent.clear();
		}
		
		mutex.unlock();
		
		dataToSend = prepareDataForViewer();
		
		ret = senderSocket.send(dataToSend,InetAddress(pViewerAddress,pViewerPort));
		
		if (ret == -1)
		{
			ERR("Error when sending message to PViewer." << endl);
		}
		
#ifdef DEBUG_MODE
		if (isPrintTimeGlobal)
		{
			cerr << "\033[22;37;1m[PTracking] MultiRobot (" << theRobotInfo.number << ") -> [" << Utils::roundN(globalBallEstimation.multiRobotX / 1000.0,2)
				 << "," << Utils::roundN(globalBallEstimation.multiRobotY / 1000.0,2) << "], validity = " << globalBallEstimation.isMultiRobotValid
				 << ", variance = " << globalBallEstimation.multiRobotVariance << "\033[0m" << endl;
		}
#endif
	}
}

vector<PoseParticleVector> GlobalEstimator::updateBestParticles(const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimationsWithModel)
{
	bestParticles.clear();
	
	for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimationsWithModel.begin(); it != estimationsWithModel.end(); ++it)
	{
		bestParticles.push_back(Utils::samplingParticles(it->second.first.observation.getCartesian(),it->second.first.sigma,bestParticlesNumber));
	}
	
	return bestParticles;
}

void GlobalEstimator::updateTargetPosition(const map<int,pair<ObjectSensorReading::Observation,Point2f> >& estimationsWithModel)
{
	estimatedTargetModels.clear();
	
	for (map<int,pair<ObjectSensorReading::Observation,Point2f> >::const_iterator it = estimationsWithModel.begin(); it != estimationsWithModel.end(); it++)
	{
		estimatedTargetModels.insert(*it);
	}
}

void GlobalEstimator::waitAgentMessages()
{
	ObjectSensorReadingMultiAgent objectSensorReadingMultiAgent;
	UdpSocket receiverSocket;
	InetAddress sender;
	string dataReceived;
	int ret;
	bool binding;
	
	do
	{
		usleep(100e3);
	}
	while (theRobotInfo.number == 0);
	
	binding = receiverSocket.bind(agentPort);
	
	if (!binding)
	{
		ERR("Error during the binding operation. Data Fusion among robots is not possible...exiting!" << endl);
		
		exit(-1);
	}
	
	WARN("Agent " << agentId << " bound on port: " << agentPort << endl);
	
	objectSensorReadingMultiAgent.setSensor(objectSensorReading.getSensor());
	
	while (true)
	{
		ret = receiverSocket.recv(dataReceived,sender);
		
		if (ret == -1)
		{
			ERR("Error in receiving message from: '" << sender.toString() << "'" << endl);
			
			continue;
		}
		
		AgentPacket ap;
		
		ap.setData(dataReceived.substr(dataReceived.find(" ") + 1));
		
		objectSensorReadingMultiAgent.setEstimationsWithModels(ap.dataPacket.estimatedTargetModels);
		objectSensorReadingMultiAgent.setEstimationsTimestamp(ap.dataPacket.particlesTimestamp);
		
		mutex.lock();
		
		observationsMultiAgent.push_back(objectSensorReadingMultiAgent);
		
		mutex.unlock();
	}
}

MAKE_MODULE(GlobalEstimator, SPQR-Modules)
