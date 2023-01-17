#include <iostream>
#include <vector>
#include "SaP.h"

using namespace std;

int main() {
	ColCal_Point a(1.0, 2.0, 3.0);
	ColCal_Point b(3.0, 2.0, 1.0);
	ColCal_Point c(3.0, 3.0, 3.0);
	ColCal_Tri tri(a, b, c, 1);
	ColCal_Box box1 = ColCal_Box(tri);
	b = a;
	cout << b.x << " " << b.y << " " << b.z;
}