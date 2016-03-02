#include "ofApp.h"

int main() {
//    ofSetupOpenGL(680, 480, OF_WINDOW);
    ofSetupOpenGL(880, 480, OF_WINDOW); // a little taller to include the depth image in addition to the other one
    
    /* 
     I don't understand how to implement multiple windows other than adding two of them like this
     but they don't have unique names or addresses or indices that I can find, so I don't get it     
     */
    
	ofRunApp(new ofApp());
}
