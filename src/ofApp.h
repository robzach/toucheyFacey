// updated very slightly 9-29-16 to attempt to turn off auto exposure (failed so far)
// updated 10/10/16 to add ofxSMTP Gmail sending functionality
// updated 10/20/16 added HSB color matching option, variety of small tweaks

#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"
#include "ofxGui.h"
#include "ofxSMTP.h"


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
    ofxCvGrayscaleImage fingerThresh;
    ofxCvGrayscaleImage dilatedThresh; // the dialated thresholded image
    
    
    // target color for color detection; approximates a yellow piece of paper
    int rTarget = 204;
    int gTarget = 183;
    int bTarget = 34;
    
    int xOfPixelWithClosestColor;
    int yOfPixelWithClosestColor;
    
    ofxFloatSlider colorThreshold;
    ofxFloatSlider minContourArea;
    ofxFloatSlider maxContourArea;
    ofxFloatSlider minBrightness;
    ofxFloatSlider maxBrightness;
    ofxFloatSlider minSaturation;
    ofxFloatSlider maxSaturation;
    ofxColorSlider sliderColor; // doesn't currently work as of 10/5/16, can't find the problem
    ofxToggle HSBmode; // added for choosing between RGB and HSB color modes on 10/19/16
    ofxToggle sendOSC;
    
    ofxPanel gui;
    
    float targetHue = 150.0; // blue default
        
    bool paintColor = false;
    bool pickColorMode = false;
    
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
    
    // no idea if this is doing what it's supposed to, see the
    // "static freenect_flag_value auto_exposure = FREENECT_ON;"
    // line in ofApp.cpp to see why it's here
    freenect_device *f_dev;
    
    // ofxSMTP variables
    void onSMTPDelivery(ofx::SMTP::Message::SharedPtr& message);
    void onSMTPException(const ofx::SMTP::ErrorArgs& evt);
    
    void onSSLClientVerificationError(Poco::Net::VerificationErrorArgs& args);
    void onSSLPrivateKeyPassphraseRequired(std::string& passphrase);
    
    std::string recipientEmail;
    std::string senderEmail;
    
    ofx::SMTP::Client smtp;
    
    string emailMessageText = "Hello face owner!\n\nAttached is the .ply file of your face, as touched by someone else at The Glitter Box in Pittsburgh. If you don't already have it, Meshlab is open-source software what can be used to open and manipulate .ply files, delete wayward vertices, export in other formats, etc. You can get it at <http://meshlab.sourceforge.net/>\n\nThanks sincerely for participating and I hope you enjoyed the experience. I'd be interested to hear any questions or comments you have as Touchey Facey is definitely a work in progress. Please see this Github page <https://github.com/robzach/toucheyFacey> for more information about the system including the source code, and here's a short video that explains it in less technical terms: <https://www.youtube.com/watch?v=bVH5vHlovxc>\n\nIf you end up making anything interesting with the attached data I'd appreciate it if you let me know! (I'm pretty curious what people will do with their faces.)\n\nThanks again,\n\nRobert Zacharias / Touchey Facey";

};
