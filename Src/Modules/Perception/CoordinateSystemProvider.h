/**
* @file CoordinateSystemProvider.h
* This file declares a module that provides coordinate systems.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Tools/Debugging/DebugImages.h"

MODULE(CoordinateSystemProvider)
  REQUIRES(Image) // for debugging only
  REQUIRES(FrameInfo)
  REQUIRES(FilteredJointData) // for timeStamp only
  REQUIRES(CameraInfo)
  REQUIRES(CameraMatrix)
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(ImageCoordinateSystem);
  LOADS_PARAMETER(float, imageRecordingTime) /**< Time the camera requires to take an image (in s, for motion compensation, may depend on exposure). */
  LOADS_PARAMETER(float, imageRecordingDelay) /**< Delay after the camera took an image (in s, for motion compensation). */
END_MODULE

class CoordinateSystemProvider : public CoordinateSystemProviderBase
{
  /**
  * Updates the image coordinate system provided by this module.
  */
  void update(ImageCoordinateSystem& imageCoordinateSystem);

  /**
  * The method calculates the scaling factors for the distored image.
  * @param a The constant part of the equation for motion distortion will be returned here.
  * @param b The linear part of the equation for motion distortion will be returned here.
  */
  void calcScaleFactors(float& a, float& b, unsigned int abTimeDiff) const;

  CameraMatrix cameraMatrixPrev[CameraInfo::numOfCameras];
  unsigned int cameraMatrixPrevTimeStamp[CameraInfo::numOfCameras];

  DECLARE_DEBUG_IMAGE(corrected);
  DECLARE_DEBUG_IMAGE(horizonAligned);
};
