#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

//--------------------------------------------------------------


void ofApp::setup() {

    fbo.allocate(ofGetWidth(), ofGetHeight());
    ofClear(255, 255, 255);
    
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    kinect.init();
    kinect.open();
    
    ofSetVerticalSync(true);
    ofBackground(25, 25, 25);
    ofEnableAlphaBlending();
    ofEnableAntiAliasing();
    ofEnableSmoothing();
    ofSetCircleResolution(100);
    
    p.set( ofGetWidth() * 0.5f, ofGetHeight() * 0.5f );
    
    // Thresholds
    nearThreshold = 255; //the higher the better
    farThreshold = 150; //the lower the better
    
    colorImg.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
    grayImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayThreshNear.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayThreshFar.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayPreprocImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    contourFinder.setFindHoles(false);
    ofSetFrameRate(30);
    showLabels = true;
    contourFinder.setMinAreaRadius(10);
    contourFinder.setMaxAreaRadius(300);
    contourFinder.setThreshold(105);
    // wait for half a second before forgetting something
    contourFinder.getTracker().setPersistence(15);
    // an object can move up to 32 pixels per frame
    contourFinder.getTracker().setMaximumDistance(32);
    step = 2;
    contourIndex = 0;
    
    ofDisableArbTex();
    ofLoadImage(texture, "circle.png");
    lineMesh.setMode(OF_PRIMITIVE_POINTS);
    texture.allocate(50, 50, OF_IMAGE_COLOR);
    bFluctuate = false;
    time = ofGetElapsedTimef();
    pointSize = 2;
}

//--------------------------------------------------------------
#pragma GCC push_options
#pragma GCC optimize ("O0")
#pragma clang optimize off
void ofApp::update() {
//    lineMesh.clear();
    
    kinect.update();
    kinectInput = kinect.isConnected();
    kinectC = char(kinectInput);
    
    

    
    switch(kinectInput){
          case 0:
            throw std::exception();
            break;
        case -1:
            throw std::exception();
            break;
    }

    if (kinect.isFrameNew()) {
        grayImage.setFromPixels(kinect.getDepthPixels());
        threshold(grayImage, grayThreshNear, nearThreshold, true);
        threshold(grayImage, grayThreshFar, farThreshold);
        Mat grayThreshNearMat = toCv(grayThreshNear);
        Mat grayThreshFarMat = toCv(grayThreshFar);
        Mat grayImageMat = toCv(grayImage);
        bitwise_and(grayThreshNearMat, grayThreshFarMat, grayImageMat);
        grayPreprocImage = grayImage;
        dilate(grayImage);
        dilate(grayImage);
        erode(grayImage);
        grayImage.update();
        contourFinder.findContours(grayImage);
    }
    
    pp = contourFinder.getPolylines();
    pathPolys = contourFinder.getPolylines();
    
    trackVelocity();
    curFlow = &fb;

}
#pragma clang optimize on
#pragma GCC pop_options
   
//--------------------------------------------------------------

void ofApp::draw() {

    ofSetColor( 0, 0, 0 );
    glScalef(-2.05,2.05,2.05);
    ofTranslate(-600, 20, 0);
    glPointSize(pointSize);

    for (int i = 0; i < meshVector.size(); i++){
        meshVector[i].setMode(OF_PRIMITIVE_POINTS);
        meshVector[i].draw();
    }
    
    createPath();
}


void ofApp::trackVelocity(){
    for (int i = 0; i < contourFinder.size(); i++){
      
        ofMesh tempMesh;
        ofPolyline polyNew = contourFinder.getPolyline(i);
        
        polyNew = polyNew.getSmoothed(50);
        col.r = ofRandom(0,1);
        col.g = ofRandom(239, 240);
        col.b = ofRandom(244,255);
        
        for (int x = 0; x < polyNew.size(); x++){
                tempMesh.addColor(col);
                tempMesh.addVertex(polyNew[x]);
        }
            
        const std::size_t MAX_BUFFER_SIZE = 100;
            
        if (meshVector.size() < MAX_BUFFER_SIZE){
            meshVector.push_back(tempMesh);
               nextIndexToWrite = meshVector.size();
        }else{
            meshVector[nextIndexToWrite] = tempMesh;
        }
        nextIndexToWrite = (nextIndexToWrite + 1) % MAX_BUFFER_SIZE;
    }
}


void ofApp::createPath(){
    ofPath newPath;
    
    for(int i = 0; i < pp.size(); i+=1) {
        pathPolys[i] = pathPolys[i].getSmoothed(50);
        for(int j = 0; j < pathPolys[i].size(); j++) {
            if(j == 0) {
                newPath.newSubPath();
                newPath.moveTo(pathPolys[i][j]);
            }else {
                newPath.lineTo(pathPolys[i][j]);
            }
        }
        newPath.close();
        newPath.setColor(ofColor(0,255,255));
    }
     newPath.draw();
}



