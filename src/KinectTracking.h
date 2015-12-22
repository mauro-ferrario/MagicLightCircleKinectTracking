//
//  KinectTracking.h
//  MagicLightCircle
//
//  Created by Mauro Ferrario on 12/11/15.
//
//

#ifndef __MagicLightCircle__KinectTracking__
#define __MagicLightCircle__KinectTracking__

#define DRAW_MODE

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
  ofParameter<int>      tiltAngle;
  int                   angle;
  cv::Mat               cam_mat;
  vector<ofVec2f>       orderedPoints;
  ofParameterGroup*     getParameterGroup();
  ofParameterGroup*     kinectTrackingParams;
  ofParameter<ofVec2f>  roiSize;
  ofParameter<ofVec2f>  roiPos;
  ofParameter<int>      maxPointToSend;
  ofParameter<float>    maxRadius;
  ofParameter<float>    radiusFromCenter;
  ofParameter<bool>     bDraw;
  ofParameter<int>      drawGrayValue;
  ofParameter<int>      drawSize;
  ofParameter<bool>     bDistanceFromCenter;
  ofParameter<bool>     bDistanceFromBaricentro;
  ofParameter<bool>     useCenterNotMaxDistance;
  ofRectangle           roiRect;
  vector<ofVec3f>       points;
  int                   width;
  int                   height;
  ofImage               drawImage;
  ofFbo                 drawFbo;
  void                  keyReleased(int key);

private:
  void                  orderPoints();
  void                  setPoints();
  void                  setupOSC();
  void                  sendOSC();
  ofxOscSender          sender;
  ofxKinect             kinect;
};

#endif /* defined(__MagicLightCircle__KinectTracking__) */
