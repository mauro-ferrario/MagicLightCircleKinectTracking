#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
  ofBackground(0);
  tracking.setup();
  gui.setup("GUI");
  gui.add(*tracking.getParameterGroup());
  gui.setPosition(ofPoint(640,0));
  gui.loadFromFile("settings.xml");
}

//--------------------------------------------------------------
void ofApp::update(){
  tracking.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
  tracking.draw();
  gui.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
  if(key == 's')
  {
    gui.saveToFile("settings.xml");
  }
  tracking.keyReleased(key);
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
