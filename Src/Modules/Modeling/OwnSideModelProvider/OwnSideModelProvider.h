/**
 * @file OwnSideModelProvider.h
 * The file declares a module that determines whether the robot cannot have left its own
 * side since the last kick-off.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Modeling/Odometer.h"

MODULE(OwnSideModelProvider)
  REQUIRES(FieldDimensions)
  REQUIRES(FrameInfo)
  REQUIRES(GameInfo)
  REQUIRES(OwnTeamInfo)
  REQUIRES(RobotInfo)
  REQUIRES(GroundContactState)
  REQUIRES(Odometer)
  PROVIDES_WITH_MODIFY(OwnSideModel)
  LOADS_PARAMETER(float, distanceUncertaintyOffset) /**< Estimated odometry and localization base error (mm). */
  LOADS_PARAMETER(float, distanceUncertaintyFactor) /**< Estimated odometry error as a factor of the distance walked. */
  LOADS_PARAMETER(float, largestXInInitial) /**< The largest x coordinate of a robot in initial (mm). */
  LOADS_PARAMETER(float, awayFromLineDistance) /**< The distance the robot has to be before or behind a certain line (mm). */
  LOADS_PARAMETER(float, minPenaltyTime) /**< The minimum time a robot must be penalized to actually believe it (ms). */
END_MODULE

/**
 * @class OwnSideModelProvider
 * A module that determines whetehr the robot cannot have left its own
 * side since the last kick-off.
 */
class OwnSideModelProvider : public OwnSideModelProviderBase
{
private:
  int lastPenalty; /**< Was the robot penalised in the last frame? */
  int lastGameState; /**< The game state in the last frame. */
  float distanceWalkedAtKnownPosition; /**< The robot walked this far at its last known position. */
  float largestXPossibleAtKnownPosition; /**< The largest x coordinate possible at its last known position. */
  bool manuallyPlaced; /**< Was the robot manually placed in the set state? */
  unsigned timeWhenPenalized; /**< When was the robot penalized. */
  int gameStateWhenPenalized; /**< What was the game state when the robot was penalized? */

  void update(OwnSideModel& ownSideModel);

public:
  OwnSideModelProvider();
};
