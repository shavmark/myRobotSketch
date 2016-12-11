#include "ofApp.h"
#include "lowlevel.h"
#include <algorithm> 
#define _USE_MATH_DEFINES
#include <math.h>

//--------------------------------------------------------------
void ofApp::draw(){
	robot.draw();
}
//--------------------------------------------------------------
void ofApp::setup() {

	robot.setup();
}

//--------------------------------------------------------------
void ofApp::update() {
	static int i = 0;
	if (i == 0) {
		ofRobotArmCommand cmd(PenPose);
		robot.arms[0]->add(ofRobotArmCommand(PenPose));
	}
	i = 1;
	return;
	if (robot.makerbots.size() > 0) {
		int count = 10;
		float slice = 2 * M_PI / count;
		ofVec2f point;
		xyDataToSend data(RobotArtists::PolyLineStream);
		float r = 0.1f;
		data.parameters.push_back(count); // lots of data coming, but its read 2 at a time by the driver
		
		for (int i = 0; i < count; i++) {
			float theta = slice * i;
			point.x = r*cos(theta);
			point.y = r*sin(theta);
			data.parameters.push_back(point.x*100);
			data.parameters.push_back(point.y*100);
		}
		robot.makerbots[0]->add(data);
		//bugbug figure out rotate, size, push/pop the canvas like OF does
		
		//robot.makerbots[0]->add(RobotArtists::PolyLineStream, 0.09f);
		return;
		
//bugbug just add a command		robot.makerbots[0]->circleMacro(0.05f);
		robot.makerbots[0]->add(RobotArtists::xyMove, ofVec2f(0.5f, 0.5f));
		robot.makerbots[0]->add(RobotArtists::xyMove, ofVec2f(.2f, 0.2f));
		robot.makerbots[0]->add(RobotArtists::xyMove, ofVec2f(0.5f, 0.5f));
		robot.makerbots[0]->add(RobotArtists::xyMove, ofVec2f(.2f, 0.2f));
		
	}
	return;//just test xy for now
	if (robot.arms.size() > 0) {
		//HighLevelTest
		ofRobotArmCommand cmd(RegressionTest);
		robot.arms[0]->add(cmd);
		/*
		ofRobotCommand cmd(RobotMoveTo, RobotCommandData(ofRobotPosition(0.0f)));
		robot->commands.add(cmd);
		cmd.reset(RobotLineTo, RobotCommandData(ofRobotPosition(0.5f)));
		robot->commands.add(cmd);
		*/
	}
	robot.update();

	//if (robotPincher.commands) {
	//	robotPincher.commands->add(ofRobotCommand::LowLevelTest);//bugbug data is too bug, just testing getting data around
		//robot.commands->add(ofRobotCommand(ofRobotCommand::Circle, 0.1f));//bugbug data is too bug, just testing getting data around
		//robot.commands->add(ofRobotCommand(ofRobotCommand::Sleep, 1000));
		//}
	//shared_ptr<RobotArtists::ofRobotCommands> cmd = robot.add(RobotArtists::ofRobotCommands::UserDefined);

	/*
	shared_ptr<RobotCommands> cmd = robot.createAndAdd<RobotCommands>(RobotCommand());
	cmd->reset(); // home the device
	cmd->setup(0.0001, 0, 0.0, 0.0, 0.0); // far left

	shared_ptr<rectangleCommand> cmd2 = robot.createCommand<rectangleCommand>();
	cmd2->reset(); // home the device
	cmd2->millisSleep = 10000;// wait before moving
	cmd2->setup(1.0, 0, 0.0, 0.0, 0.0); // far right
	robot.add(cmd2);

	shared_ptr<rectangleCommand> cmd3 = robot.createCommand<rectangleCommand>();
	cmd3->reset(); // home the device
	cmd3->millisSleep = 10000;// wait before moving
	cmd3->setup(0.0, 1.0, 0.0, 0.0, 0.0);
	robot.add(cmd3);

	shared_ptr<rectangleCommand> cmd4 = robot.createCommand<rectangleCommand>();
	cmd4->reset(); // home the device
	cmd4->millisSleep = 10000;// wait before moving
	cmd4->setup(0.0, 0.0001, 0.0, 0.0, 0.0);
	robot.add(cmd4);

	//motion->setup(); // start a new motion bugbug outside of testing  like now this is only done one time
	//path.push(motion);
	//shared_ptr<RobotMotion> joints = make_shared<RobotMotion>(serial, data);
	//joints->setup(SignOnDance);
	//path.push(joints);
	/*
	return;
	data.setCommand(EnableArmMovement);
	data.setY(50);
	path.push(data);
	data.reset();
	data.setY(350);
	path.push(data);
	return;
	data.reset();
	data.setX(10); // 300 max 60 units covers about 3", 20 units is .75" so given the arch its not just inches
	data.setY(350);
	path.push(data);
	data.reset();
	data.setZ(250);
	data.setX(-10); // -300 max
	path.push(data);
	data.reset();
	data.setCommand(DelayArm, 1000);
	path.push(data);

	loc.reset();
	loc.moveYout(260);
	loc.moveZup(250);
	path.push(loc);
	loc.reset();
	loc.moveXleft(-212);
	path.push(loc);
	loc.reset();
	loc.moveXleft(200);
	loc.moveYout(200);
	path.push(loc);
	*/

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
