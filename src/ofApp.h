#pragma once

#include "ofMain.h"
#include "lowlevel.h"
#include "ofUtils.h"
#include "ofRobots.h"

using namespace RobotArtists;

// bugbug openframeworks specific items here, c++ only above this line. use of as prefix as needed


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		vector<string> arguments;

		ofRobot robot;
};
