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

ofVec2f               centerToConsiderForSorting;

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
  cout << "*************" << endl;
  for(int a = 0; a < points.size(); a++)
  {
    if(a < maxPointToSend)
    {
      ofVec3f sendPoint = points[a];
      // Quando si inviano le coordinate considerando il baricentro, forse bisognerebbe rimappare le coordinate e riportar le coordinate relative al boundingBox della sagoma, nelle proporzioni delle coordinate relative al boundingBox del ROI
      sendPoint.x = 1 - float((points[a].x - roiRect.x)/(roiRect.width));
      sendPoint.y = float((points[a].y - roiRect.y)/(roiRect.height));
      int color;
#ifdef DRAW_MODE
      color = drawImage.getColor(points[a].x, points[a].y).r;
#else
      // Testare
      int index = points[a].y * width + points[a].x;
      color = kinect.getDepthPixels()[index];
#endif
//      cout << "COLOR = "  << color << endl;
      sendPoint.z =  color;
      ofxOscMessage m;
      m.setAddress("/newPoint/"+ofToString(a));
      m.addFloatArg(sendPoint.x);
      m.addFloatArg(sendPoint.y);
      m.addFloatArg(sendPoint.z);
      sender.sendMessage(m);
      cout << "SEND POINT " << a << endl;
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
  width   = 640;
  height  = 480;
#ifdef DRAW_MODE
  setupDrawMode();
#else
  setupKinect();
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

void KinectTracking::setupDrawMode()
{
  drawImage.allocate(width, height, OF_IMAGE_COLOR);
  drawFbo.allocate(width, height);
  drawFbo.begin();
  ofClear(0,255);
  drawFbo.end();
}

void KinectTracking::setupKinect()
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
  angle = 0;
  kinect.setCameraTiltAngle(angle);
}

void KinectTracking::updateROI()
{
  roiRect.x = roiPos->x;
  roiRect.y = roiPos->y;
  roiRect.width = roiSize->x;
  roiRect.height = roiSize->y;
  roiRect.width = ofClamp(roiRect.width, 0, kinect.width - roiPos->x);
  roiRect.height = ofClamp(roiRect.height, 0, kinect.height - roiPos->y);
  roiCenter.x = roiRect.x + (roiRect.width * .5);
  roiCenter.y = roiRect.y + (roiRect.height * .5);
}

void KinectTracking::update()
{
  updateROI();
  unsigned char * pixels;
#ifndef DRAW_MODE
  kinect.setCameraTiltAngle(tiltAngle);
  kinect.update();
  if(kinect.isFrameNew())
  {
    pixels = kinect.getDepthPixels();
#else
    if(bDraw)
      updateDraw();
    pixels = drawImage.getPixels();
#endif
    updateContourFinder(pixels);
    points.clear();
    if(contourFinder.getContours().size()>0)
    {
      if(!useCenterNotMaxDistance||bDistanceFromBaricentro)
      {
        orderPoints();
        setPoints();
      }
      else
      {
        checkBlobPointsFromTheCenter();
      }
    }
#ifndef DRAW_MODE
  }
#endif
  if(bDistanceFromBaricentro||bDistanceFromCenter)
    sendOSC();
}

void KinectTracking::checkBlobPointsFromTheCenter()
{
  int totPointFound = 0;
  for(int a = 0; a < contourFinder.getContours().size();a++)
  {
    if(totPointFound < maxPointToSend)
    {
      ofPoint point = ofPoint(contourFinder.getCentroid(a).x, contourFinder.getCentroid(a).y);
      point.x += roiRect.x;
      point.y +=  roiRect.y;
      if(point.distance(roiCenter) > roiSize->x * radiusFromCenter)
      {
        totPointFound++;
        points.push_back(point);
      }
    }
  }
}

void KinectTracking::updateDraw()
{
  drawFbo.begin();
  ofPushStyle();
  ofFill();
  ofSetColor(drawGrayValue);
  ofCircle(ofGetMouseX(), ofGetMouseY(), drawSize);
  ofPopStyle();
  drawFbo.end();
  drawFbo.readToPixels(drawImage.getPixelsRef());
  drawImage.update();
  drawImage.setImageType(OF_IMAGE_GRAYSCALE);
}

void KinectTracking::updateContourFinder( unsigned char * pixels)
{
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
  centerToConsiderForSorting = roiCenter;
  if(bDistanceFromBaricentro)
    centerToConsiderForSorting = ofPoint(contourFinder.getCentroid(0).x + roiRect.x, contourFinder.getCentroid(0).y + roiRect.y);
  ofSort(orderedPoints,ordina);
}

void KinectTracking::setPoints()
{
  // Al momento setta solamente al massimo 2 punti... farlo per anche più punti per utilizzare braccia, gambe, testa, tentacoli....
  bool secondPointFound = false;
  // Funziona con un solo blob/sagoma
  ofPoint centerToConsider = roiCenter;
  if(bDistanceFromBaricentro)
    centerToConsider = ofPoint(contourFinder.getCentroid(0).x + roiRect.x, contourFinder.getCentroid(0).y + roiRect.y);
  if(orderedPoints.size()>0)
  {
    ofVec3f point;
    int idMaxDistancePoint = 0;
    float distanceFromCenter;
    int cont = 0;
    do
    {
      point.x = orderedPoints[cont].x;
      point.y = orderedPoints[cont].y;
      point.z = 0;
      distanceFromCenter = centerToConsider.distance(point);
      if(cont > orderedPoints.size())
        return;
      idMaxDistancePoint = cont;
      cont++;
    }while(distanceFromCenter < roiSize->x * radiusFromCenter);
    points.push_back(point);
    
    point = orderedPoints[cont];
    int totPoints = orderedPoints.size();
    float distanceFromFirstPoint = orderedPoints[idMaxDistancePoint].distance(point);
    while(abs(distanceFromFirstPoint) <= maxRadius)
    {
      cont++;
      point = orderedPoints[cont];
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
      ofPoint contourCenter = ofPoint(contourFinder.getCenter(a).x,contourFinder.getCenter(a).y);
      ofSetColor(255,0,0);
      ofCircle(contourCenter,10);
    }
    ofPopStyle();
  }
  ofPopMatrix();
  ofSetColor(255,0,0);
  ofRect(roiRect.x,roiRect.y,roiRect.width, roiRect.height);
  if(bDistanceFromCenter)
  {
    ofCircle(roiCenter, 10);
    ofCircle(roiCenter, radiusFromCenter * roiSize->x);
  }
  if(bDistanceFromBaricentro||bDistanceFromCenter)
    drawFoundPoints();
  ofPopMatrix();
  ofPopMatrix();
}

void KinectTracking::drawFoundPoints()
{
  ofPoint centerToConsider = roiCenter;
  if(bDistanceFromBaricentro&&contourFinder.getContours().size()>0)
    centerToConsider = ofPoint(contourFinder.getCentroid(0).x + roiRect.x, contourFinder.getCentroid(0).y + roiRect.y);
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