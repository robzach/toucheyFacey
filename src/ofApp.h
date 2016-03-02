#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

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
    
    
    void recordFacePoint(int xTouch, int yTouch);
    void exportFacePoints();
    void drawFaceCloud();
    
	ofxKinect kinect;
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
		
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
    bool bDrawFaceCloud;
    
    ofMesh faceMesh; // global mesh that will contain all the points added to it by recordFacePoints;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
};
