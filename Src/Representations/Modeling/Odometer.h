/*
 * @file Odometer.h
 *
 * Some additional odometry information
 *
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
 * @author marcel
 */

#pragma once

#include "Tools/Math/Pose2D.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(Odometer,
{,
  (float)(10000.f) distanceWalked,  /** Total distance walked since start of B-Human software */
  (Pose2D) odometryOffset, /** Odometry difference since last frame */
});
