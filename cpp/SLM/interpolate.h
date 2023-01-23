#pragma once
#include <gtsam/base/Vector.h>

namespace gtsam{

Vector2 interpolate(double x0, double y0, double x1, double y1, int coeff) {
double new_x = 1;
return Vector2 (new_x, new_x);
}

}