#pragma once

#include "ofMain.h"
#include "ofMain.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxWatchdog.h"


class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void fluctuate();

    void keyPressed(int key);
    
    void switchFluc(int speed);
    
    
    
    ofxKinect kinect;
    ofxCv::ContourFinder contourFinder;
    
    ofImage colorImg;
    ofImage grayImage;         // grayscale depth image
    ofImage grayThreshNear;    // the near thresholded image
    ofImage grayThreshFar;     // the far thresholded image
    ofImage grayPreprocImage;  // grayscale pre-processed image
    
    int nearThreshold;
    int farThreshold;
    int angle;
    
    
    // used for viewing the point cloud
    ofEasyCam easyCam;
    bool showLabels;
    cv::Rect recttt;
    
    ofVec2f p;
    ofColor col;
    
    int step;
    int contourIndex;
    
    
    
    ofPolyline polyline;
    ofPolyline smoothPoly;
    ofPath pathFromContour;//path to be built
    std::vector<ofPolyline> pp;
    ofFbo fbo;
    
    ofVec2f velocity;
    ofMesh lineMesh;
    ofVboMesh vboMesh;

    
    ofTexture texture; //this will be used to load a texture onto the dots that we are drawing on the screen
    ofPath path;
    ofMesh m;
        int counter;
    ofPolyline polyTest;
    ofMesh normals;
    bool bFluctuate;
    
    
    int startTime;
    int endTime;
    
    float time;
    int kinectInput;
    char kinectC;
private:
    of3dPrimitive prim;
    ofBoxPrimitive geometry;
};
