//
//  KinectTracking.h
//  MagicLightCircle
//
//  Created by Mauro Ferrario on 12/11/15.
//
//

#ifndef __MagicLightCircle__KinectTracking__
#define __MagicLightCircle__KinectTracking__

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

class KinectTracking
{
public:
                        KinectTracking();
                        ~KinectTracking();
  void                  setup();
  void                  update();
  void                  draw();
  void                  exit();
  cv::Mat               gerROIImage();
  ofxKinect             kinect;
  ofxCvColorImage       colorImg;
  ofxCvGrayscaleImage   grayImage;
  ofxCvGrayscaleImage   grayImage2;
  ofxCvGrayscaleImage   grayThreshNear;
  ofxCvGrayscaleImage   grayThreshFar;
  ofxCv::ContourFinder  contourFinder;
  ofParameter<int>      nearThreshold;
  ofParameter<int>      farThreshold;
  ofParameter<float>    minArea;
  ofParameter<float>    maxArea;
  int                   angle;
  cv::Mat               cam_mat;
  vector<ofVec2f>       orderedPoints;
  ofParameterGroup*     getParameterGroup();
  ofParameterGroup*     kinectTrackingParams;
  ofParameter<ofVec2f>  roiSize;
  ofParameter<ofVec2f>  roiPos;
  ofParameter<int>      maxPointToSend;
  ofParameter<float>    maxRadius;
  ofRectangle roiRect;
  vector<ofVec3f>       points;

private:
  void                  orderPoints();
  void                  setPoints();
  void                  setupOSC();
  void                  sendOSC();
  ofxOscSender          sender;
};

#endif /* defined(__MagicLightCircle__KinectTracking__) */
