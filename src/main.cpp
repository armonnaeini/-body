#include "ofMain.h"
#include "ofApp.h"
#include "ofxWatchdog.h"


//========================================================================
int main( ){
	ofSetupOpenGL(1700,1000,OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofxWatchdog::boot(3000, true, true, true);
	ofxWatchdog::trap();


	ofRunApp(new ofApp());


}
