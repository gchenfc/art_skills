#include "../SlnStroke.h"
#include <gtsam/geometry/Point2.h>
#include <vector>
#include <fstream>

using namespace std;
using namespace gtsam;
using namespace art_skills;


int main( int argc, char* argv[] ) {
  // Create the csv file we will use to plot the GTSAM result
  std::ofstream myfile;
  myfile.open ("stroke1_005_60.csv");

  //Substitute in the results from GTSAM (good estimate):
  const Point2 xy(0.328137741848, -0.891717473651);
  const double t0 = -0.446723503295;
  const double D = 0.589139027037;
  const double theta1 = 1.23570533014;
  const double theta2 = 1.46710580947;
  const double sigma = 0.0186252618364;
  const double mu = 1.31957471217;
  const double dt = 0.0042;

  //Substitute in the results from GTSAM (good estimate):
  const Point2 xy(0.328137741848, -0.891717473651);
  const double t0 = -3.59840229207;
  const double D = 0.624518665631;
  const double theta1 = 1.23174322281;
  const double theta2 = 1.46710580947;
  const double sigma = 0.0186252618364;
  const double mu = 1.31957471217;
  const double dt = 0.0042;

  // Create the stroke
  //const SlnStroke stroke(xy, t0, D, theta1, theta2, sigma, mu);
  const SlnStroke stroke(xy, t0, D, theta1, theta2, sigma, mu);
  // populate headers
  myfile << "time,x,y\n";
  for(size_t k=1; k<60; k++) {
      double k_t = k * dt;
      Point2 pt = stroke.position(k_t, dt);
      //cout << "px is: " << pt(0) << endl;
      myfile << k_t << "," << pt(0) << "," << pt(1) << "\n";
  }
  myfile.close();
  return 0;
}
