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
    ofBackground(70, 70, 70);
    ofEnableDepthTest();
    ofEnableAlphaBlending(); //enable transparancy
    ofEnableAntiAliasing(); //enable anti-aliasing
    ofEnableSmoothing(); //enable line smooth
    ofSetCircleResolution(1000); //nicer circles
    
    p.set( ofGetWidth() * 0.5f, ofGetHeight() * 0.5f );
    
    // Thresholds
    nearThreshold = 255; //230;
    farThreshold = 160; //70;
    colorImg.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
    grayImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayThreshNear.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayThreshFar.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    grayPreprocImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    contourFinder.setFindHoles(false);
    ofSetFrameRate(60);
    showLabels = true;
    contourFinder.setMinAreaRadius(1);
    contourFinder.setMaxAreaRadius(300);
    contourFinder.setThreshold(205);
    // wait for half a second before forgetting something
    contourFinder.getTracker().setPersistence(15);
    // an object can move up to 32 pixels per frame
    contourFinder.getTracker().setMaximumDistance(32);
    step = 2;
    contourIndex = 0;
    
    ofDisableArbTex();
    ofLoadImage(texture, "circle.png");
        lineMesh.setMode(OF_PRIMITIVE_POINTS);

    texture.allocate(10, 10, OF_IMAGE_COLOR);
    

    bFluctuate = false;


    time = ofGetElapsedTimef();

    glPointSize(2);





}

//--------------------------------------------------------------
#pragma GCC push_options
#pragma GCC optimize ("O0")
#pragma clang optimize off
void ofApp::update() {
    
    
    ofBackground(15, 15, 15);
    kinect.update();
    
    kinectInput = kinect.isConnected(); //location may be off
    
    kinectC = char(kinectInput); //convert binary to char
//    cout << kinectInput << endl;
    
    switch(kinectInput){
          case 0:
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
    ofColor col = (ofRandom(255), ofRandom(255), ofRandom(255));
        
   


    for (int i = 0; i < contourFinder.size(); i++){
        ofVec2f velocity = toOf(contourFinder.getVelocity(i));
        int velocityX = abs(velocity.x);
        int velocityY = abs(velocity.y);

        if (velocityX > 2 || velocityY > 2)
            bFluctuate = true;
        else
            bFluctuate = false;
    }

    
    /*clear mesh*/

    if (bFluctuate == false){

        for(int i = 0; i < pp.size(); i++) {
//                cout << contourFinder.getVelocity(i) << endl;
                pp[i] = pp[i].getSmoothed(25);

                for (int j = 0; j < pp[i].size(); j+=1){
                    lineMesh.addVertex(pp[i][j]);
                    lineMesh.addColor(255);
                }
        }

         if (ofGetFrameNum() % 20 == 0){
                lineMesh.clear();
         }
    }
    
    
  





}
#pragma clang optimize on
#pragma GCC pop_options
   

    


//--------------------------------------------------------------

void ofApp::draw() {
    
    ofSetColor( 255, 255, 255 );
    
    glScalef(-2.55,2.55,2.55);
    ofTranslate(-850, 0, 0);
    
    
    
   
   

    if(bFluctuate) {
        fluctuate();
    }

    lineMesh.draw();


//    for(auto polyline: pp)
//        for (auto point: polyline)
//            texture.draw(point);

}

void ofApp::fluctuate(){

        for(int i = 0; i < pp.size(); i++) {
            pp[i] = pp[i].getSmoothed(25);

            /*
                   for (int j = 0; j < pp[i].size(); j+=1){ //iterates through a single polyline, within vector, around 50000-100000
                       ofVec3f rotatedPoint(0,0,0);
                       rotatedPoint.x = pp[i][j].x; //current point's x
                       rotatedPoint.y = pp[i][j].y; //current point's y

                       float angleOffset = (float)j/(float)pp[i].size() * TWO_PI; //ANGLE OFFSET NECESSARY FOR OFSETTING POLYLINE
                       float scale = 75.0 * sin(angleOffset + counter);
                       pp[i][j] = rotatedPoint + (scale); //iterates through all specific points, as the points are being displaced
                   }*/

            for (int j = 0; j < pp[i].size(); j+=1){ //iterates through a single polyline, within vector, around 50000-100000
                col.r = 250;
                col.g = 92;
                col.b = 190;

                lineMesh.addColor(col);
                lineMesh.addVertex(pp[i][j]);
            }
        }
}


void ofApp::switchFluc(int speed){
}



void ofApp::keyPressed (int key) {
    switch (key) {
        case'f':
            bFluctuate = !bFluctuate;
            break;
    }
}
