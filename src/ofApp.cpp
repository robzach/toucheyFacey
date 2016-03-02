#include "ofApp.h"

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init(); //false by default
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    
	// print the intrinsic IR sensor values
    
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
    
	colorImg.allocate(kinect.width, kinect.height);
    cout << "kinect.width=" << kinect.width <<", kinect.height= " << kinect.height << endl;
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
//	angle = 0;
//	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    ofBackground(100, 100, 100);

}

//--------------------------------------------------------------
void ofApp::update() {
	
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
        ofPixels pixels = kinect.getDepthPixels() ;
        
		grayImage.setFromPixels(pixels);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
//		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
    
    
    // RIGHT HERE IS A GREAT PLACE TO ADD THE PROCESS THAT IDENTIFIES THE CLOSEST COLOR MATCH PIXEL
    
    /* 
     
     as an intermediate: use the current mouse position (easy to do) as the "finger" location
     so later the finger tracking can be added in
     
     */
    
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	}
    
    // WOULD BE NICE FOR THIS TO BE IN ITS OWN WINDOW BUT I CAN'T FIGURE OUT HOW TO DO THAT
    else if(bDrawFaceCloud){
        easyCam.begin();
        drawFaceCloud();
        easyCam.end();
    }
    
    else {
		// draw from the live kinect
		kinect.draw(0, 0, 640, 480); // main RGB image
        kinect.drawDepth(640, 0, 200, 150); // depth image, rendered smaller to the side
		
//		grayImage.draw(10, 320, 400, 300);
//		contourFinder.draw(10, 320, 400, 300);
		
	}
	
	// draw instructions
	stringstream reportStream;
    
    {
//    if(kinect.hasAccelControl()) {
//        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
//        << ofToString(kinect.getMksAccel().y, 2) << " / "
//        << ofToString(kinect.getMksAccel().z, 2) << endl;
//    } else {
//        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
//		<< "motor / led / accel controls are not currently supported" << endl << endl;
//    }
    
//	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
//	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
//	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
//	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << endl //contourFinder.nBlobs
//	<< ", fps: " << ofGetFrameRate() << endl
//	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

//    if(kinect.hasCamTiltControl()) {
//    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
//        << "press 1-5 & 0 to change the led mode" << endl;
//    }
    } // prior code commented out
    
    reportStream << "RGB image on left," << endl << "depth image in upper right" << endl;
    
	ofDrawBitmapString(reportStream.str(), 640, 450);
    
}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
                ofColor color1 = kinect.getColorAt(x,y) ;
				mesh.addColor(color1);
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(5);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

void ofApp::recordFacePoint(int xTouch, int yTouch){
    cout << "recording a face point now at:\n"
    << "mouseX = " << mouseX << endl
    << "mouseY = " << mouseY << endl;
    
    faceMesh.setMode(OF_PRIMITIVE_POINTS);
    faceMesh.addVertex(kinect.getWorldCoordinateAt(mouseX, mouseY));
    
    
    /*
     locate the point labeled with the color marker using the RGB camera
     and then append its real-world XYZ from the Kinect to a list
     display the points on this list
     */
    
    /*
     call a function from here that finds the winning location point in the RGB image (closest color to the intended one, inside of an acceptable range)
     then turn that RGB image location into the real-world XYZ image with mesh.addVertex(kinect.getWorldCoordinateAt(x, y)) or something like that
     */
    
    
}

void ofApp::drawFaceCloud() { // function mostly copied from drawPointCloud()
    
    // should run only once when called because otherwise just keeps piling points on top of each other
    
    
    
    int w = 640;
    int h = 480;
    faceMesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
//    for(int y = 0; y < h; y += step) {
//        for(int x = 0; x < w; x += step) {
//            if(kinect.getDistanceAt(x, y) > 0) {
//                ofColor red(255, 0, 0); // draw face points in red, this is arbitrary
//                faceMesh.addColor(red);
//                faceMesh.addVertex(kinect.getWorldCoordinateAt(x, y));
//            }
//        }
//    }
    
    ofColor red(255, 0, 0); // draw face points in red, this is arbitrary
    glPointSize(5);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    ofEnableDepthTest();
    faceMesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}

void ofApp::exportFacePoints(){
    //export a bunch of points as a JSON or OF mesh or whatever format is appropriate to a file
    
}

//--------------------------------------------------------------
void ofApp::exit() {
//	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
            
        // this is the keystroke that will be triggered by MakeyMakey
        case 'a':
            recordFacePoint(mouseX, mouseY);
            // maybe recordFacePoint() should take the current mouse position as an argument? Nah, just have the function query that itself probably
            // it appears I changed my mind and wanted to pass the mouse position in.
            break;
            
		case ' ':
			faceMesh.clearVertices();
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
            
        case'f':
            bDrawFaceCloud = !bDrawFaceCloud;
            break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}


//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
    cout << "click\n"; // just testing
    // here: need to identify the color where the click happens
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}