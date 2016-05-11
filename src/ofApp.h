#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"
#include "ofxGui.h"

#define HOST "localhost"
#define PORT 12345

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
    
    void recordFacePoint(float xTouch, float yTouch);
    void exportFacePoints();
    void drawFaceCloud();
    
	ofxKinect kinect;
	
	ofxCvColorImage colorImg;
    
    ofColor clickColor;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
//    ofImage fingerThresh; // the thresholded image for the finger marked with a colored band
    ofxCvGrayscaleImage fingerThresh;
    ofxCvGrayscaleImage dilatedThresh; // the dialated thresholded image
    
    
    // target color for color detection; values from a poorly lit sample attempt
    int rTarget = 204;
    int gTarget = 183;
    int bTarget = 34;
    
    int xOfPixelWithClosestColor;
    int yOfPixelWithClosestColor;
    
//    int colorThreshold = 50;
    ofxFloatSlider colorThreshold;
    ofxFloatSlider minContourArea;
    ofxFloatSlider maxContourArea;
    
    ofxPanel gui;
        
    bool paintColor = false;
    bool buildThresh = true;
    bool pickColorMode = false;
    bool sendOSC = false;
    
    ofxCvContourFinder contourFinder;
    ofxCvBlob blob;
    
    ofFbo fbo;
    
    ofMesh faceMesh; // global mesh that will contain all the points added to it by recordFacePoints;
	
	float angle;
    
    string faceSavePath = "/Users/rz/Documents/CMU/IACD/touchey_facey non-OF/faces";
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
    
    ofSoundPlayer ping;
    
    ofxOscSender sender;
    
    ofRectangle roiMat = ofRectangle(640/3, 480/3, 640/3, 640/3);

};
