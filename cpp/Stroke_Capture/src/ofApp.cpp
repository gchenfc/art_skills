#include "ofApp.h"
#include "hanning.hpp"

#define HANNING_SIZE 41

inline float norm2(ofPoint p) {
	return sqrt(p.x * p.x + p.y * p.y);
}

inline float dot2(ofPoint p1, ofPoint p2) {
	return p1.x * p2.x + p1.y * p2.y;
}

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetFrameRate(200);
	ofSetVerticalSync(false);

	smoother = hanning(HANNING_SIZE, 0);
	/*
	for (int i = 0; i < 41; i++) {
		cout << i << " " << smoother[i] << endl;
	}
	*/
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);

	ofSetColor(255);

	for (int i = 0; i < lines.size(); i ++) {
		for (int j = 0; j < lines[i].size() - 1; j++) {

			if (drawing && i == lines.size() - 1) {
				ofSetColor(0, 255, 255);
			}

			if (showSmoothed) {
				if (i < smooth_lines.size()) {
					ofDrawLine(smooth_lines[i][j], smooth_lines[i][j + 1]);
				}

				if (drawing && i == lines.size() - 1) {
					ofDrawLine(lines[i][j], lines[i][j + 1]);
				}
			}
			else {
				ofDrawLine(lines[i][j], lines[i][j + 1]);
			}
		}
	}

	// visualize curvature
	//if (curvature.size() > 0) {
	//	ofPushMatrix();
	//	ofTranslate(20, 700);

	//	int sz = curvature[curvature.size() - 1].size();

	//	ofSetColor(255);
	//	ofDrawLine(0, 0, sz, 0);

	//	ofSetColor(0, 255, 255);
	//	for (int i = 0; i < sz - 1; i++) {
	//		ofDrawLine(i - 1, -curvature[curvature.size() - 1][i].z * 100000,
	//			i, -curvature[curvature.size() - 1][i + 1].z * 100000);
	//	}

	//	ofPopMatrix();
	//}

	// visualize velocity // TODO: change to velocity from "speed"
	if (velocity.size() > 0) {
		ofPushMatrix();
		ofTranslate(20, 700);

		int sz = velocity[velocity.size() - 1].size();

		ofSetColor(255, 255, 0);
		for (int i = 0; i < sz - 1; i++) {
			ofDrawLine(i - 1, -velocity[velocity.size() - 1][i] * 100,
				i, -velocity[velocity.size() - 1][i + 1] * 100);
		}

		ofPopMatrix();
	}

	// visualize vdot
	//if (vdot.size() > 0) {
	//	ofPushMatrix();
	//	ofTranslate(20, 700);

	//	int sz = vdot[vdot.size() - 1].size();

	//	ofSetColor(255, 0, 255);
	//	for (int i = 0; i < sz - 1; i++) {
	//		ofDrawLine(i - 1, -vdot[vdot.size() - 1][i] * 1000,
	//			i, -vdot[vdot.size() - 1][i + 1] * 1000);
	//	}

	//	ofPopMatrix();
	//}

	// visualize vddot
	if (vddot.size() > 0) {
		ofPushMatrix();
		ofTranslate(20, 700);

		int sz = vddot[vddot.size() - 1].size();

		ofSetColor(0, 255, 255);
		for (int i = 0; i < sz - 1; i++) {
			ofDrawLine(i - 1, -vddot[vddot.size() - 1][i] * 10000,
				i, -vddot[vddot.size() - 1][i + 1] * 10000);
		}

		ofPopMatrix();
	}

	ofSetColor(0, 255, 0);
	int printY = 20, dY = 18;
	ofDrawBitmapString("framerate: " + ofToString(ofGetFrameRate()), 20, printY);
	ofDrawBitmapString("[s]moothed trajectories: " + ofToString(showSmoothed), 20, printY+=dY);
	ofDrawBitmapString("[r]set", 20, printY += dY);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 's') {
		showSmoothed = !showSmoothed;
	}
	if (key == 'r') {
		for (int i = 0; i < lines.size(); i++) {
			lines[i].clear();
		}
		lines.clear();

		for (int i = 0; i < smooth_lines.size(); i++) {
			smooth_lines[i].clear();
		}
		smooth_lines.clear();

		for (int i = 0; i < curvature.size(); i++) {
			curvature[i].clear();
		}
		curvature.clear();

		for (int i = 0; i < velocity.size(); i++) {
			velocity[i].clear();
		}
		velocity.clear();

		for (int i = 0; i < salient_points.size(); i++) {
			salient_points[i].clear();
		}
		salient_points.clear();

		for (int i = 0; i < timestamps.size(); i++) {
			timestamps[i].clear();
		}
		timestamps.clear();
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
	if (button == 0 && drawing == true) {

		ofPoint p(x, y); // TODO: collect timestamp

		//cout << lines[lines.size() - 1].size() << " : " << p << " " << ofGetSystemTimeMillis() << endl;

		lines[lines.size() - 1].push_back(p);
		timestamps[timestamps.size() - 1].push_back(ofGetSystemTimeMillis());
	}

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	if (button == 0 && drawing == false) {

		vector<ofPoint> l;
		ofPoint p(x, y); // TODO: collect timestamp

		vector<long long> t;

		cout << endl << "[ " << (lines.size()) << " th trajectory ]" << endl;
		cout << l.size() << " :  " << p << " " << ofGetSystemTimeMillis() << endl;

		l.push_back(p);
		lines.push_back(l);
		t.push_back(ofGetSystemTimeMillis());
		timestamps.push_back(t);

		drawing = true;
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
	if (button == 0) {
		drawing = false;

		// smooth filter 
		if (lines.size() > smooth_lines.size()) {
			vector<ofPoint> sl;

			//cout << endl << "[ " << (lines.size() - 1) << " th trajectory (smoothed) ]" << endl;

			int v = lines.size() - 1;
			int sz = lines[lines.size() - 1].size();

			for (int i = 0; i < sz; i++) {
				int n = HANNING_SIZE / 2;

				ofPoint p; // TODO: collect timestamp

				for (int j = 0; j <= n; j++) { // prior half + current
					int idx = i - n + j;

					if (idx < 0) idx = 0;
					if (idx > sz - 1) idx = sz - 1;

					p += smoother[j] * lines[v][idx];
				}

				for (int j = n + 1; j < HANNING_SIZE; j++) { // later half
					int idx = i - n + j;

					if (idx < 0) idx = 0;
					if (idx > sz - 1) idx = sz - 1;

					p += smoother[j] * lines[v][idx];
				}

				// cout << i << " : " << p << endl;

				sl.push_back(p);
			}

			smooth_lines.push_back(sl);


			// curvature computation
			vector<ofPoint> cl;

			//cout << endl << "[ " << (lines.size() - 1) << " th curvature ]" << endl;

			for (int i = 0; i < sz; i++) {
				if (i == 0) { // first point
					ofPoint p(sl[i]);
					cl.push_back(p);
				}
				else if (i == sz - 1) { // last point
					ofPoint p(sl[i]);
					cl.push_back(p);
				}
				else {
					ofPoint v1 = sl[i] - sl[i - 1];
					ofPoint v2 = sl[i + 1] - sl[i];

					float theta = 1 - dot2(v1, v2) / (norm2(v1) * norm2(v2)); //what theta is this? 

					ofPoint p(sl[i].x, sl[i].y, theta);
					cl.push_back(p);

					//cout << i << " : " << theta << endl;
				}
			}

			curvature.push_back(cl);
			
			// velocity computation
			vector<float> vl;

			//cout << endl << "[ " << (lines.size() - 1) << " th velocity ]" << endl;

			for (int i = 0; i < sz; i++) {
				float speed = 0;
				if (i < sz - 1) {
					speed = sl[i].distance(sl[i + 1]) / (timestamps[v][i + 1] - timestamps[v][i]);
					cout << "stroke # "<< v << " time " << (timestamps[v][i + 1] - timestamps[v][i]) << " ? " << smooth_lines[v][i].distance(smooth_lines[v][i + 1]) << endl;
				}
				vl.push_back(speed);
			}

			velocity.push_back(vl);


			// vdot computation
			vector<float> vdl;

			for (int i = 0; i < sz; i++) {
				float speed_dot = 0;
				if (i < sz - 1) {
					speed_dot = (vl[i+1] - vl[i]) / (timestamps[v][i + 1] - timestamps[v][i]);
					//cout << "stroke # " << v << " time " << (timestamps[v][i + 1] - timestamps[v][i]) << " ? " << smooth_lines[v][i].distance(smooth_lines[v][i + 1]) << endl;
				}
				vdl.push_back(speed_dot);
			}

			vdot.push_back(vdl);

			//// vddot computation
			vector<float> vddl;

			for (int i = 0; i < sz; i++) {
				float speed_ddot = 0;
				if (i < sz - 1) {
					speed_ddot = (vdl[i + 1] - vdl[i]) / (timestamps[v][i + 1] - timestamps[v][i]);
					//cout << "stroke # " << v << " time " << (timestamps[v][i + 1] - timestamps[v][i]) << " ? " << smooth_lines[v][i].distance(smooth_lines[v][i + 1]) << endl;
				}
				vddl.push_back(speed_ddot);
			}

			vdot.push_back(vddl);

			// local maxima/minima -> salient points
			
			//const char* path = "/Users/JDFlorez/Documents/art_skills/matlab/velocity.txt";
			//std::ofstream f(path);

			// Send data to the stream
			//for (vector<float>::const_iterator i = vl.begin(); i != vl.end(); ++i)
			//{
			//	f << *i << '\n';
			//}

			// Close the file
			//f.close();

			//cout << endl << "[ " << (lines.size() - 1) << " th DONE ]" << endl;
			//cout << endl << "[ " << ofGetSystemTimeMillis() << " TIME ]" << endl;

		}
	}
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}

//--------------------------------------------------------------
//--------------------------------------------------------------