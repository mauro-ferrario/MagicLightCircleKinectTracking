//
//  KinectTracking.cpp
//  MagicLightCircle
//
//  Created by Mauro Ferrario on 12/11/15.
//
//

#include "KinectTracking.h"

using namespace ofxCv;
using namespace cv;

ofVec2f center;

bool ordina(const ofVec2f &a, const ofVec2f &b){
  return a.distance(center) > b.distance(center);
}

KinectTracking::KinectTracking()
{
  
}

KinectTracking::~KinectTracking()
{
  
}

void KinectTracking::setup()
{
  kinect.setRegistration(true);
  kinect.init();
  kinect.open();
  if(kinect.isConnected())
  {
    ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
    ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
    ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
    ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
  }
  colorImg.allocate(kinect.width, kinect.height);
  grayImage.allocate(kinect.width, kinect.height);
  grayThreshNear.allocate(kinect.width, kinect.height);
  grayThreshFar.allocate(kinect.width, kinect.height);
  nearThreshold = 230;
  farThreshold = 70;
  ofSetFrameRate(60);
  angle = 0;
  kinect.setCameraTiltAngle(angle);
  nearThreshold = 218;
  farThreshold = 157;
}

void KinectTracking::update()
{
  roiRect.x = roiPos->x;
  roiRect.y = roiPos->y;
  roiRect.width = roiSize->x;
  roiRect.height = roiSize->y;
  roiRect.width = ofClamp(roiRect.width, 0, kinect.width - roiPos->x);
  roiRect.height = ofClamp(roiRect.height, 0, kinect.height - roiPos->y);
  kinect.update();
  if(kinect.isFrameNew())
  {
    contourFinder.setMinArea(minArea);
    contourFinder.setMaxArea(maxArea);
    grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
    grayThreshNear = grayImage;
    grayThreshFar = grayImage;
    grayThreshNear.threshold(nearThreshold, true);
    grayThreshFar.threshold(farThreshold);
    cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
    grayImage.flagImageChanged();
    contourFinder.findContours(gerROIImage());
    orderPoints();
    setPoints();
  }
}

void KinectTracking::orderPoints()
{
  orderedPoints.clear();
  vector< vector<cv::Point> > points = contourFinder.getContours();
  for(int a = 0; a < points.size(); a++)
  {
    for(int i = 0; i < points[a].size(); i++)
    {
      orderedPoints.push_back(ofVec2f(points[a][i].x,points[a][i].y));
    }
  }
  ofSort(orderedPoints,ordina);
}

void KinectTracking::setPoints()
{
  points.clear();
  if(orderedPoints.size()>0)
  {
    ofVec3f point;
    point.x = orderedPoints[0].x;
    point.y = orderedPoints[0].y;
    point.z = kinect.getDistanceAt(point.x + roiRect.x, point.y + roiRect.y);
    points.push_back(point);
    int cont = 1;
    point = orderedPoints[cont];
    point.z = kinect.getDistanceAt(point.x, point.y);
    int totPoints = orderedPoints.size();
    float distanceFromFirstPoint = orderedPoints[0].distance(point);
    while(abs(distanceFromFirstPoint) < maxRadius)
    {
      cont++;
      point = orderedPoints[cont];
      point.z = kinect.getDistanceAt(point.x + roiRect.x, point.y+ roiRect.y);
      distanceFromFirstPoint = orderedPoints[0].distance(point);
    }
    if(point.x > 0 && point.x < roiRect.width && point.y > 0 && point.y < roiRect.height)
      points.push_back(point);
  }

}

cv::Mat KinectTracking::gerROIImage()
{
  cv::Mat crop;
  cam_mat = toCv(grayImage);
  cv::Rect crop_roi = cv::Rect(roiRect.x, roiRect.y, roiRect.width, roiRect.height);
  crop = cam_mat(crop_roi).clone();
  center.x = roiRect.x + (roiRect.width * .5);
  center.y = roiRect.y + (roiRect.height * .5);
  return crop;
}

void KinectTracking::exit()
{
  kinect.setCameraTiltAngle(0);
  kinect.close();
}

void KinectTracking::draw()
{
  float width = kinect.width;
  float height = kinect.height;
  ofSetColor(255);
  kinect.drawDepth(0, 0, width, height);
  ofSetColor(255,0,0,100);
  grayImage.draw(0, 0, width, height);
  ofPushStyle();
  ofPopStyle();
  ofNoFill();
  ofSetColor(255);
  ofRect(0,0, width, height);
  ofPushMatrix();
  ofPushMatrix();
  ofTranslate(roiRect.x, roiRect.y);
  ofSetColor(255,255,0);
  contourFinder.draw();
  ofPopMatrix();
  ofPushStyle();
  ofSetColor(255,0,0);
  ofRect(roiRect.x,roiRect.y,roiRect.width, roiRect.height);
  ofCircle(center, 10);
  ofSetLineWidth(2);
  for(int a = 0; a < points.size(); a++)
  {
    ofLine(center.x, center.y, points[a].x + roiRect.x, points[a].y+roiRect.y);
    if(a < points.size()-1)
      ofCircle(points[a].x + roiRect.x, points[a].y+roiRect.y, maxRadius);
  }
  ofPopStyle();
  ofPopMatrix();
}

ofParameterGroup* KinectTracking::getParameterGroup()
{
  if(!kinectTrackingParams)
  {
    kinectTrackingParams = new ofParameterGroup();
  }
  if(kinectTrackingParams->getName() == "")
  {
    kinectTrackingParams->setName("KinectTracking");
    kinectTrackingParams->add(roiPos.set("Roi pos", ofVec2f(0,0), ofVec2f(0,0), ofVec2f(kinect.width, kinect.height)));
    kinectTrackingParams->add(roiSize.set("Roi size", ofVec2f(10,10), ofVec2f(10,10), ofVec2f(kinect.width, kinect.height)));
    kinectTrackingParams->add(nearThreshold.set("Near Threshold", 218,0, 255));
    kinectTrackingParams->add(farThreshold.set("Far Threshold", 110,0, 255));
    kinectTrackingParams->add(minArea.set("Min area", 110,0, 1000));
    kinectTrackingParams->add(maxArea.set("Max area", 500,0, kinect.width * kinect.height));
    kinectTrackingParams->add(maxRadius.set("Max radius", 200,0, 500));
    kinectTrackingParams->add(maxPointToSend.set("Max point to send", 2,0, 10));
  }
  return kinectTrackingParams;
}