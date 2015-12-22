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
ofVec2f centerToConsiderForSorting;

bool ordina(const ofVec2f &a, const ofVec2f &b){
  return a.distance(centerToConsiderForSorting) > b.distance(centerToConsiderForSorting);
}

KinectTracking::KinectTracking()
{
  
}

KinectTracking::~KinectTracking()
{
  
}

void KinectTracking::setupOSC()
{
  sender.setup("localhost", 12345);
}

void KinectTracking::sendOSC()
{
  for(int a = 0; a < points.size(); a++)
  {
    if(a < maxPointToSend)
    {
      ofVec3f sendPoint = points[a];
      sendPoint.x = 1 - float((points[a].x - roiRect.x)/(roiRect.width));
      sendPoint.y = float((points[a].y - roiRect.y)/(roiRect.height));
      sendPoint.z =  255;
      ofxOscMessage m;
      m.setAddress("/newPoint/"+ofToString(a));
      m.addFloatArg(sendPoint.x);
      m.addFloatArg(sendPoint.y);
      m.addFloatArg(sendPoint.z);
      sender.sendMessage(m);
    }
  }
}

void KinectTracking::keyReleased(int key)
{
  if(key == 'd')
  {
    bDraw = !bDraw;
  }
}

void KinectTracking::setup()
{
#ifdef DRAW_MODE
  width   = 640;
  height  = 480;
  drawImage.allocate(width, height, OF_IMAGE_COLOR);
  drawFbo.allocate(width, height);
  drawFbo.begin();
  ofClear(0,255);
  drawFbo.end();
#else
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
  width = kinect.width;
  height = kinect.height;
  angle = 0;
  kinect.setCameraTiltAngle(angle);
#endif
  
  colorImg.allocate(width, height);
  grayImage.allocate(width, height);
  grayThreshNear.allocate(width, height);
  grayThreshFar.allocate(width, height);
  nearThreshold = 230;
  farThreshold = 70;
  ofSetFrameRate(60);
  nearThreshold = 218;
  farThreshold = 157;
  setupOSC();
}

void KinectTracking::update()
{
  roiRect.x = roiPos->x;
  roiRect.y = roiPos->y;
  roiRect.width = roiSize->x;
  roiRect.height = roiSize->y;
  roiRect.width = ofClamp(roiRect.width, 0, kinect.width - roiPos->x);
  roiRect.height = ofClamp(roiRect.height, 0, kinect.height - roiPos->y);
  center.x = roiRect.x + (roiRect.width * .5);
  center.y = roiRect.y + (roiRect.height * .5);
  unsigned char * pixels;
#ifndef DRAW_MODE
  kinect.setCameraTiltAngle(tiltAngle);
  kinect.update();
  if(kinect.isFrameNew())
  {
    pixels = kinect.getDepthPixels();
#else
    drawFbo.begin();
    if(bDraw)
    {
      ofPushStyle();
      ofFill();
      ofSetColor(drawGrayValue);
      ofCircle(ofGetMouseX(), ofGetMouseY(), drawSize);
      ofPopStyle();
    }
    drawFbo.end();
    drawFbo.readToPixels(drawImage.getPixelsRef());
    drawImage.update();
    drawImage.setImageType(OF_IMAGE_GRAYSCALE);
    pixels = drawImage.getPixels();
#endif
    contourFinder.setMinArea(minArea);
    contourFinder.setMaxArea(maxArea);
    grayImage.setFromPixels(pixels, width, height);
    grayThreshNear = grayImage;
    grayThreshFar = grayImage;
    grayThreshNear.threshold(nearThreshold, true);
    grayThreshFar.threshold(farThreshold);
    cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
    grayImage.flagImageChanged();
    contourFinder.findContours(gerROIImage());
    if(contourFinder.getContours().size()>0)
    {
      if(!useCenterNotMaxDistance||bDistanceFromBaricentro)
      {
        orderPoints();
        setPoints();
      }
      else
      {
        points.clear();
        int totPointFound = 0;
        for(int a = 0; a < contourFinder.getContours().size();a++)
        {
          if(totPointFound < maxPointToSend)
          {
            ofPoint point = ofPoint(contourFinder.getCentroid(a).x, contourFinder.getCentroid(a).y);
            point.x += roiRect.x;
            point.y +=  roiRect.y;
            if(point.distance(center) > roiSize->x * radiusFromCenter)
            {
              totPointFound++;
              points.push_back(point);
            }
          }
        }
      }
    }
#ifndef DRAW_MODE
  }
#endif
  sendOSC();
}

void KinectTracking::orderPoints()
{
  orderedPoints.clear();
  vector< vector<cv::Point> > points = contourFinder.getContours();
  for(int a = 0; a < points.size(); a++)
  {
    for(int i = 0; i < points[a].size(); i++)
    {
      orderedPoints.push_back(ofVec2f(points[a][i].x + roiRect.x,points[a][i].y + roiRect.y));
    }
  }
  centerToConsiderForSorting = center;
  if(bDistanceFromBaricentro)
    centerToConsiderForSorting = ofPoint(contourFinder.getCentroid(0).x + roiRect.x, contourFinder.getCentroid(0).y + roiRect.y);
  ofSort(orderedPoints,ordina);
}

void KinectTracking::setPoints()
{
  // Al momento setta solamente al massimo 2 punti... farlo per anche più punti per utilizzare braccia, gambe, testa, tentacoli....
  points.clear();
  bool secondPointFound = false;
  // Funziona con un solo blob/sagoma
  ofPoint centerToConsider = center;
  if(bDistanceFromBaricentro)
    centerToConsider = ofPoint(contourFinder.getCentroid(0).x + roiRect.x, contourFinder.getCentroid(0).y + roiRect.y);
  if(orderedPoints.size()>0)
  {
    ofVec3f point;
    int idMaxDistancePoint = 0;
    float distanceFromCenter;
    
    int cont = 0;
    point.x = orderedPoints[cont].x;
    point.y = orderedPoints[cont].y;
    point.z = 0; // kinect.getDistanceAt(point.x, point.y);
    distanceFromCenter = centerToConsider.distance(point);
    
    while(distanceFromCenter < roiSize->x * radiusFromCenter)
    {
      cont++;
      point.x = orderedPoints[cont].x;
      point.y = orderedPoints[cont].y;
      point.z = 0; // kinect.getDistanceAt(point.x, point.y);
      distanceFromCenter = centerToConsider.distance(point);
      if(cont > orderedPoints.size())
        return;
    }
    points.push_back(point);
    idMaxDistancePoint = cont;
    cont++;
    point = orderedPoints[cont];
//    point.z = kinect.getDistanceAt(point.x, point.y);
    int totPoints = orderedPoints.size();
    float distanceFromFirstPoint = orderedPoints[idMaxDistancePoint].distance(point);
    while(abs(distanceFromFirstPoint) <= maxRadius)
    {
      cont++;
      point = orderedPoints[cont];
//      point.z = kinect.getDistanceAt(point.x, point.y);
      distanceFromFirstPoint = orderedPoints[idMaxDistancePoint].distance(point);
      distanceFromCenter = centerToConsider.distance(point);
      if((abs(distanceFromFirstPoint) > maxRadius&&cont < totPoints) && distanceFromCenter > roiSize->x * radiusFromCenter)
        secondPointFound = true;
    }
    if(secondPointFound&&(point.x > roiRect.x && point.x < roiRect.x + roiRect.width && point.y > roiRect.y && point.y < roiRect.y + roiRect.height))
      if(maxPointToSend > 1)
        points.push_back(point);
  }
}

cv::Mat KinectTracking::gerROIImage()
{
  cv::Mat crop;
  cam_mat = toCv(grayImage);
  cv::Rect crop_roi = cv::Rect(roiRect.x, roiRect.y, roiRect.width, roiRect.height);
  crop = cam_mat(crop_roi).clone();
  return crop;
}

void KinectTracking::exit()
{
#ifndef DRAW_MODE
  kinect.setCameraTiltAngle(0);
  kinect.close();
#endif
}

void KinectTracking::draw()
{
  ofPushMatrix();
  ofSetColor(255);
#ifndef DRAW_MODE
  ofTranslate(width,0);
  ofScale(-1,1);
  kinect.drawDepth(0, 0, width, height);
#else
//  drawImage.draw(0,0);
  drawFbo.draw(0,0);
#endif
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
  
  
  if(bDistanceFromBaricentro)
  {
    ofPushStyle();
    ofFill();
    ofSetColor(0,0,255);
    for(int a = 0; a < contourFinder.getContours().size(); a++)
    {
      ofCircle(contourFinder.getCentroid(a).x, contourFinder.getCentroid(a).y,10);
      ofNoFill();
      ofCircle(contourFinder.getCentroid(a).x, contourFinder.getCentroid(a).y, radiusFromCenter * roiSize->x);
//      ofCircle(contourFinder.getCenter(a).x, contourFinder.getCenter(a).y,10);
    }
    ofPopStyle();
  }
  
  if(bDistanceFromBaricentro)
  {
    ofPushStyle();
    ofSetColor(255,0,0);
    for(int a = 0; a < contourFinder.getContours().size(); a++)
    {
      ofPoint contourCenter = ofPoint(contourFinder.getCenter(a).x,contourFinder.getCenter(a).y);
      ofCircle(contourCenter,10);
    }
    ofPopStyle();
  }
  
  ofPopMatrix();
  ofPushStyle();
  ofSetColor(255,0,0);
  ofRect(roiRect.x,roiRect.y,roiRect.width, roiRect.height);
  
  
  
  if(bDistanceFromCenter)
  {
    ofCircle(center, 10);
    ofCircle(center, radiusFromCenter * roiSize->x);
  }
  
  ofPoint centerToConsider = center;
  if(bDistanceFromBaricentro&&contourFinder.getContours().size()>0)
  {
    centerToConsider = ofPoint(contourFinder.getCentroid(0).x + roiRect.x, contourFinder.getCentroid(0).y + roiRect.y);
  }
  ofSetLineWidth(2);
  for(int a = 0; a < points.size(); a++)
  {
    if(points[a].x > roiRect.x && points[a].x < roiRect.x + roiRect.width && points[a].y > roiRect.y && points[a].y < roiRect.y + roiRect.height)
    {
      ofLine(centerToConsider.x, centerToConsider.y, points[a].x, points[a].y);
      ofDrawBitmapString(ofToString(a), points[a].x, points[a].y);
    }
    if(a < points.size()-1)
      ofCircle(points[a].x, points[a].y, maxRadius);
    ofPushStyle();
    ofSetColor(255,255,0);
    ofFill();
    ofCircle(points[a].x, points[a].y, 15);
    ofPopStyle();
  }
  ofPopStyle();
  ofPopMatrix();
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
    kinectTrackingParams->add(roiPos.set("Roi pos", ofVec2f(0,0), ofVec2f(0,0), ofVec2f(width, height)));
    kinectTrackingParams->add(roiSize.set("Roi size", ofVec2f(10,10), ofVec2f(10,10), ofVec2f(width, height)));
    kinectTrackingParams->add(nearThreshold.set("Near Threshold", 218,0, 255));
    kinectTrackingParams->add(farThreshold.set("Far Threshold", 110,0, 255));
    kinectTrackingParams->add(minArea.set("Min area", 110,0, 1000));
    kinectTrackingParams->add(maxArea.set("Max area", 500,0, width * height));
    kinectTrackingParams->add(maxRadius.set("Max radius", 200,0, 500));
    kinectTrackingParams->add(maxPointToSend.set("Max point to send", 2,0, 10));
    kinectTrackingParams->add(tiltAngle.set("Tilt angle", 0,-30, 30));
    kinectTrackingParams->add(bDraw.set("Draw", true));
    kinectTrackingParams->add(drawGrayValue.set("Draw Gray Value",100,0, 255));
    kinectTrackingParams->add(drawSize.set("Draw size",100,10, 500));
    kinectTrackingParams->add(bDistanceFromCenter.set("Calculate point distance from center", true));
    kinectTrackingParams->add(useCenterNotMaxDistance.set("Use Center Not Max Distance", true));
    kinectTrackingParams->add(radiusFromCenter.set("Radius from center",.5,0, 1));
    kinectTrackingParams->add(bDistanceFromBaricentro.set("Calculate point distance from baricentro", true));
  }
  return kinectTrackingParams;
}