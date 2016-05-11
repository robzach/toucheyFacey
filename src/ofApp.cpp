// 5/4/16 4:28pm added gui sliders, tweaked a bit for presentation
// 5/5/16 5:27pm added ROI in center of image

#include "ofApp.h"

void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init(); //false by default
//	kinect.init(true); // shows infrared instead of RGB video image
	
	kinect.open();		// opens first available kinect
    
	colorImg.allocate(kinect.width, kinect.height);
    cout << "kinect.width=" << kinect.width <<", kinect.height= " << kinect.height << endl;
    
    fbo.allocate(640, 480, GL_RGBA);
    fbo.begin();
    ofClear(255,255,255, 0);
    fbo.end();
    
    fingerThresh.clear(); // saw some weird noise crop up (likely stray data) on loading the image, so clearing it first
    fingerThresh.allocate(640,480);
    dilatedThresh.allocate(640,480);

    faceMesh.setupIndicesAuto(); // vertices should index themselves automatically
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
//	angle = 0;
//	kinect.setCameraTiltAngle(angle);
    
    ofBackground(100);
    
    angle = kinect.getCurrentCameraTiltAngle();
    cout << "Kinect angle is " << angle << endl;
    
    ping.load("pingSound.wav");
    
    sender.setup(HOST, PORT);
    
    gui.setup();
    gui.add(colorThreshold.setup("colorThreshold", 50, 10, 100));
    gui.add(minContourArea.setup("minContourArea", 10, 1, 30));
    gui.add(maxContourArea.setup("maxContourArea", 100, 11, 200));



}

//--------------------------------------------------------------
void ofApp::update() {
	
	
	kinect.update();
    
    fbo.begin();
    ofClear(255,255,255, 0); // writes over previous vertices so they don't leave trails forever when drawing
    easyCam.begin();
    drawFaceCloud();
    easyCam.end();
    fbo.end();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
        
        if(buildThresh){
            
            unsigned char *colorData = kinect.getPixels().getData(); // gather pixel values
            unsigned char *dstArray = fingerThresh.getPixels().getData(); // buffer to load thresholded image into
            
            int nPixels = kinect.width * kinect.height;
            
            for (int i=0; i<nPixels; i++){
                
                int byteIndex = i*3;
                unsigned char r = colorData [byteIndex + 0];
                unsigned char g = colorData [byteIndex + 1];
                unsigned char b = colorData [byteIndex + 2];
                
                int dr = r - rTarget;
                int dg = g - gTarget;
                int db = b - bTarget;
                float dh = sqrt(dr*dr + dg*dg + db*db);
                
                unsigned char dstValue = (dh < colorThreshold) ? 255 : 0;
                dstArray[i] = dstValue;
            }
            
            fingerThresh.updateTexture();
            
            dilatedThresh = fingerThresh;
            dilatedThresh.dilate_3x3();
            
//            ofxCvSetImageROI(dilatedThresh, cvRect(213, 160, 213, 160)); // trying to set ROI but didn't work
            
            fingerThresh.setROI(roiMat);
            dilatedThresh.setROI(roiMat);
            
            contourFinder.findContours(dilatedThresh, minContourArea, maxContourArea, 1, FALSE);
            
            
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw() {
    ofSetColor(255);
    
    kinect.draw(0, 0, 640, 480); // main RGB image
    kinect.drawDepth(0, 0, 640*.25, 480*.25); // depth image, rendered smaller
    dilatedThresh.draw(0, 480*.25, 640*.25, 480*.25); // dialated image of fingerThresh
    
    for (int i = 0; i < contourFinder.nBlobs; i++){
        ofTranslate(640/3, 480/3); // needed because ROI shifts data points
        contourFinder.blobs[i].draw();
        ofSetColor(255,0,0);
        ofFill();
        ofDrawCircle(contourFinder.blobs[i].boundingRect.getCenter().x, contourFinder.blobs[i].boundingRect.getCenter().y, 5);
        ofTranslate(-640/3, -480/3); // unshift back
    }
    
    ofSetColor(255);
    ofNoFill();
//    ofDrawRectangle(640/3, 480/3, 640/3, 640/3); // center rectangle
    ofDrawRectangle(roiMat);
    if(paintColor) ofDrawEllipse (xOfPixelWithClosestColor, yOfPixelWithClosestColor, 10,10);
    
    fbo.draw(640,0); // changing the value of these arguments doesn't matter at all for whatever reason

    ofSetColor(0,255,0);
    
    if(pickColorMode) ofDrawBitmapString("pick color mode", 290,240);
    
    ofDrawBitmapString("kinect RGB image", 0, 480-10);
    ofDrawBitmapString("depth image", 0, (480*0.25)-5);
    ofDrawBitmapString("color selection", 0, (480*0.5)-5);
    
    ofDrawBitmapString("press p for points", 650, 480);
    ofDrawBitmapString("press x for scribble", 650, 480-10);
    ofDrawBitmapString("press l for lines", 650, 480-20);
    ofDrawBitmapString("press f to pull all points forward", 850, 480-10);
    ofDrawBitmapString("press d to push all points backward", 850, 480);
    
    if(sendOSC) ofDrawBitmapString("sending OSC data", 650,10);
    
    stringstream verticesReport;
    verticesReport << faceMesh.getNumVertices() << " vertices recorded";
    ofDrawBitmapString(verticesReport.str(), 640, 490);
    
    gui.draw();
    
}

void ofApp::recordFacePoint(float xTouch, float yTouch){
    
    ofVec3f origin; // defaults to origin without needing to be defined
    int adjustedZValue = kinect.getWorldCoordinateAt(xTouch, yTouch)[2] - 700;
    ofVec3f KinectVec(kinect.getWorldCoordinateAt(xTouch, yTouch)[0], kinect.getWorldCoordinateAt(xTouch, yTouch)[1], adjustedZValue);
    
    // only add point if it's reasonably close (at most 175 units(millimeters?)) to the origin
    if (KinectVec.squareDistance(origin) < 30625){
        
        faceMesh.addVertex(KinectVec);
        //    cout << "squareDistance calculated value is " << KinectVec.squareDistance(origin) << endl;
        
        cout << "recording a face point now at screen x, y = " << xTouch
        << ", " << yTouch << " with Kinect real world x, y, z = "
        << kinect.getWorldCoordinateAt(xTouch, yTouch)[0] << ", "
        << kinect.getWorldCoordinateAt(xTouch, yTouch)[1] << ", "
        << adjustedZValue << endl;
        
        ping.play();
        
        if (sendOSC) {
            ofxOscMessage m;
            m.setAddress("/vecPoints");
            m.addFloatArg(KinectVec[0]);
            m.addFloatArg(KinectVec[1]);
            m.addFloatArg(KinectVec[2]);
            sender.sendMessage(m, false);
            cout << "sent OSC x y z "
            << KinectVec[0] << " "
            << KinectVec[1] << " "
            << KinectVec[2] << endl;
        }
    }
}

void ofApp::drawFaceCloud() {
    
    int w = 640;
    int h = 480;
    glPointSize(7);
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofEnableDepthTest();
    faceMesh.addColor(ofColor::red); // draw points in red, this is arbitrary
    faceMesh.drawVertices();
    ofDisableDepthTest();

}

void ofApp::exportFacePoints(){
    string username = ofSystemTextBoxDialog("Whose face was touched?");
    string faceSavePath = "/Users/rz/Documents/CMU/IACD/touchey_facey non-OF/faces/" + username + "face.ply";
//    string faceSavePath = "/Users/rzachari/Desktop/faces" + username + "face.ply";

    faceMesh.save(faceSavePath);
    ofSystemAlertDialog("PLY mesh saved at " + faceSavePath);
}

void ofApp::keyPressed (int key) {
	switch (key) {
            
        // to use mouse for debugging; records points painted over by the mouse
        case 'b':
            recordFacePoint(mouseX, mouseY);
            break;
            
        // record point from contour centroid position (for usual Makey Makey operation)
        case 'a':
            // the 640/3 and 480/3 are added to undo the translation the ROI does
            recordFacePoint(contourFinder.blobs[0].boundingRect.getCenter().x+640/3, contourFinder.blobs[0].boundingRect.getCenter().y+480/3);
            cout << "contourFinder.blobs[0].boundingRect.getCenter().x , y = " << contourFinder.blobs[0].boundingRect.getCenter().x << " ," << contourFinder.blobs[0].boundingRect.getCenter().y << endl;
            break;
        
        {
        // hit n to clear all recorded vertices and transmit OSC message to clear remote display
        case 'n':
            faceMesh.clearVertices();
            if (sendOSC){
                ofxOscMessage c;
                c.setAddress("/clearMsg");
                c.addBoolArg(true);
                sender.sendMessage(c, false);
            }
            break;
        }
            
//        case'c': // turn on the color selector
//            paintColor = !paintColor;
//            break;
            
//        case'=':
//            colorThreshold++;
//            break;
//        
//        case'-':
//            colorThreshold--;
//            break;
            
        {
        // command to save the current face to a .ply file
        case's':
            exportFacePoints();
            break;
			
        }
            
        case',':
            pickColorMode = !pickColorMode;
            break;
        
        {
        // remove any (-0, 0, 0) vertices on command
        case'r':
            int removeCounter = 0;
            int numVert = faceMesh.getNumVertices();
            for (int i = 0; i < numVert; i++){
                if ( faceMesh.getVertex(i)[0] == 0 || faceMesh.getVertex(i)[1] == 0 || faceMesh.getVertex(i)[2] == 0){
                    faceMesh.removeVertex(i);
                    removeCounter++;
                }
            }
            cout << "removed " << removeCounter << " vertices with zero value" << endl;
            break;
        }
            
        { // add a short string of vertices to help align the view
        case 'v' :
            ofVec3f v1(0,0,0);
            ofVec3f v2(0,0,100);
            ofVec3f v3(0,0,500);
            ofVec3f v4(0,0,1000);
            ofColor blue(0, 0, 255);
            faceMesh.addColor(blue);
            faceMesh.addVertex(v1);
            faceMesh.addVertex(v2);
            faceMesh.addVertex(v3);
            faceMesh.addVertex(v4);
            break;
        }
    
			
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
            
        case 'x':
            faceMesh.setMode(OF_PRIMITIVE_LINE_STRIP_ADJACENCY);
            break;
            
        case 'l':
            faceMesh.setMode(OF_PRIMITIVE_LINES);
            break;
            
            
        case 'p':
            faceMesh.setMode(OF_PRIMITIVE_POINTS);
            break;
            
        {
        // pull all points forward by a distance of 5
        case 'f':
            int numVert = faceMesh.getNumVertices();
            for (int i = 0; i < numVert; i++){
                int x = faceMesh.getVertex(i)[0];
                int y = faceMesh.getVertex(i)[1];
                int newZ = faceMesh.getVertex(i)[2] - 5;
                ofVec3f pulledVec;
                pulledVec.set (x, y, newZ);
                faceMesh.setVertex(i, pulledVec);
            }
            break;
        }
            
        {
            // push all points backwards by a distance of 5
        case 'd':
            int numVert = faceMesh.getNumVertices();
            for (int i = 0; i < numVert; i++){
                int x = faceMesh.getVertex(i)[0];
                int y = faceMesh.getVertex(i)[1];
                int newZ = faceMesh.getVertex(i)[2] + 5;
                ofVec3f pulledVec;
                pulledVec.set (x, y, newZ);
                faceMesh.setVertex(i, pulledVec);
            }
            break;
        }
            
        case 'o':
            sendOSC = !sendOSC;
            break;
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
    
    if(pickColorMode){
        clickColor = kinect.getColorAt(mouseX, mouseY) ;
        cout << "color at click = " << clickColor << endl;
        rTarget = clickColor.r;
        gTarget = clickColor.g;
        bTarget = clickColor.b;
        
        cout << "rTarget = " << rTarget << endl;
        cout << "gTarget = " << gTarget << endl;
        cout << "bTarget = " << bTarget << endl;
    }
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
    
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

//--------------------------------------------------------------
void ofApp::exit() {
    //	kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
}






















//////////////////////////////// OLD STUFF //////////////////////////////////////


/*


for (int y=160; y<373 ; y++) {
    for (int x=213; x<426; x++) {

        int rArrayIndex = (y*kinect.width*3) + (x*3);
        int gArrayIndex = (y*kinect.width*3) + (x*3) + 1;
        int bArrayIndex = (y*kinect.width*3) + (x*3) + 2;

        // Now you can get and set values at location (x,y), e.g.:
        unsigned char redValueAtXY   = colorData[rArrayIndex];
        unsigned char greenValueAtXY = colorData[gArrayIndex];
        unsigned char blueValueAtXY  = colorData[bArrayIndex];

        float rDif = redValueAtXY - rTarget; // difference in reds
        float gDif = greenValueAtXY - gTarget; // difference in greens
        float bDif = blueValueAtXY - bTarget; // difference in blues

        // The Pythagorean theorem gives us the Euclidean distance.
        float colorDistance = sqrt (rDif*rDif + gDif*gDif + bDif*bDif);

        if(colorDistance < fingerThreshVal){
            fingerThresh.setColor(x, y, ofColor::white)
        }
        else fingerThresh.setColor(x, y, ofColor::black);

    }
}



 
 
 
 

for (int y=160; y<373 ; y++) {
    for (int x=213; x<426; x++) {

        ofColor thisPixel;

        // Now you can get and set values at location (x,y), e.g.:
        unsigned char redValueAtXY   = kinect.getColorAt(x, y).r;
        unsigned char greenValueAtXY = kinect.getColorAt(x, y).g;
        unsigned char blueValueAtXY  = kinect.getColorAt(x, y).b;

        float rDif = redValueAtXY - rTarget; // difference in reds
        float gDif = greenValueAtXY - gTarget; // difference in greens
        float bDif = blueValueAtXY - bTarget; // difference in blues

        // The Pythagorean theorem gives us the Euclidean distance.
        float colorDistance = sqrt (rDif*rDif + gDif*gDif + bDif*bDif);

//                    if(colorDistance < fingerThreshVal){
//                        fingerThresh.setColor(x, y, ofColor::white)
//                    }
//                    else fingerThresh.setColor(x, y, ofColor::black);

    }
}



 
 
 


if(paintColor){
    
    int leastDistanceSoFar = 25; // start with a reasonably tight threshold
    
    // only scan center of image (only real area of interest since face will be in there) 640/3, 480/3, 640/3, 640/3
    for (int y=160; y<373 ; y++) {
        for (int x=213; x<426; x++) {
            
            ofColor thisPixelColor = kinect.getColorAt(x,y) ;
            
            float rDif = thisPixelColor.r - rTarget; // difference in reds
            float gDif = thisPixelColor.g - gTarget; // difference in greens
            float bDif = thisPixelColor.b - bTarget; // difference in blues
            
            // The Pythagorean theorem gives us the Euclidean distance.
            float colorDistance =
            sqrt (rDif*rDif + gDif*gDif + bDif*bDif);
            
            if(colorDistance < leastDistanceSoFar){
                leastDistanceSoFar = colorDistance;
                xOfPixelWithClosestColor = x;
                yOfPixelWithClosestColor = y;
            }
        }
    }
}

*/
