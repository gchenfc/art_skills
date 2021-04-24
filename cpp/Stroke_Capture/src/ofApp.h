#pragma once

#include "ofMain.h"

class ofApp : public ofBaseApp{
	public:
		void setup();
		void update();
		void draw();
		
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void write_csv(std::string filename, std::vector<float> dataset);

		float* smoother;
		vector<vector<ofPoint>> lines;
		vector<vector<ofPoint>> smooth_lines;
		vector<vector<ofPoint>> curvature;
		vector<vector<float>> velocity;
		vector<vector<float>> vdot;
		vector<vector<float>> vddot;
		vector<vector<ofPoint>> salient_points;
		vector<vector<long long>> timestamps;

		bool drawing = false;
		bool showSmoothed = false;

};
