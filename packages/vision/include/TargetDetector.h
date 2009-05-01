/*
 * Copyright (C) 2009 Robotics at Maryland
 * Copyright (C) 2009 Joseph Lisee
 * All rights reserved.
 *
 * Author: Joseph Lisee <jlisee@umd.edu>
 * File:  packages/vision/include/TargetDetector.h
 */

#ifndef RAM_TARGET_DETECTOR_H_04_29_2009
#define RAM_TARGET_DETECTOR_H_04_29_2009

// Project Includes
#include "vision/include/Common.h"
#include "vision/include/Detector.h"
#include "vision/include/BlobDetector.h"

#include "core/include/ConfigNode.h"

// Must be incldued last
#include "vision/include/Export.h"

namespace ram {
namespace vision {
    
class RAM_EXPORT TargetDetector : public Detector
{
  public:
    TargetDetector(core::ConfigNode config,
                   core::EventHubPtr eventHub = core::EventHubPtr());
    ~TargetDetector();

    void processImage(Image* input, Image* output= 0);

    bool found();

    /** Center of the red light in the local horizontal, -4/3 to 4/3 */
    double getX();
    
    /** Center of the red light in the local vertical, -1 to 1 */
    double getY();

    /** The percent of the top of the image blacked out */
    void setTopRemovePercentage(double percent);

    /** The percent of the bottom of the image blacked out */
    void setBottomRemovePercentage(double percent);
        
  private:
    void init(core::ConfigNode config);

    /** Filters the image so that all green is white */
    //void filterForGreen();

    /** Find target blob candidates: use the null blobs that are inside the
     * Green blobs and combine to fill most of the space the green blob as well
     * as have centroids pretty close together.
     * Second method is to use the horizontal and vertical runner techniques
     * to attempt to find the bounds of the internal space
     */
    
    /** Processes the list of all found blobs and finds the larget valid one  */
    bool processBlobs(const BlobDetector::BlobList& blobs,
                      BlobDetector::Blob& outBlob);

    /** Publish the found event with the latest target information */
    void publishFoundEvent();
    
    /** Filters for the green in the water */
    ColorFilter* m_filter;

    /** Working scratch image */
    Image* m_image;

    /** Wether or not the target is found */
    bool m_found;
    
    /** The location for the X center of the target */
    double m_targetCenterX;

    /** The location for the Y center of the target */
    double m_targetCenterY;

    /** 1 when the area inside the target is perfectly square, < 1 otherwise */
    double m_squareNess;

    /** 0 When the target fills the screen, goes to 1 as it shrinks */
    double m_distance;

    /** The minimum pixel count of the green target blob */
    int m_minGreenPixels;

    /** How many times to erode the green filtered elements */
    int m_erodeIterations;

    /** How many times to dilate the green filtered elements */
    int m_dilateIterations;

    /** Finds the red light */
    BlobDetector m_blobDetector;

    /** How un square are blob can be and still be considered a target */
    double m_maxAspectRatio;
    
    /** Percentage of the image to remove from the top */
    double m_topRemovePercentage;

    /** Percentage of the image to remove from the bottom */
    double m_bottomRemovePercentage;
};
	
} // namespace vision
} // namespace ram

#endif // RAM_TARGET_DETECTOR_H_04_29_2009
