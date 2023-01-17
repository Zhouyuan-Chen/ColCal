#include "ColCal_Tri.h"

ColCal_Tri::ColCal_Tri() {
	this->Point[0] = ColCal_Point(0, 0, 0);
	this->Point[1] = ColCal_Point(0, 0, 0);
	this->Point[2] = ColCal_Point(0, 0, 0);
	this->idx = -1;
}

ColCal_Tri::ColCal_Tri(const ColCal_Point p1, const ColCal_Point p2, const ColCal_Point p3, int Idx) {
	this->Point[0] = p1;
	this->Point[1] = p2;
	this->Point[2] = p3;
	this->idx = Idx;
}
ColCal_Tri::ColCal_Tri(const ColCal_Tri& tri) {
	this->Point[0] = tri.Point[0];
	this->Point[1] = tri.Point[1];
	this->Point[2] = tri.Point[2];
	this->idx = tri.idx;
}

ColCal_Tri& ColCal_Tri::operator=(const ColCal_Tri& tri) {
	this->Point[0] = tri.Point[0];
	this->Point[1] = tri.Point[1];
	this->Point[2] = tri.Point[2];
	this->idx = tri.idx;

	return *this;
}

bool ColCal_Tri::ColCal_Collision(const ColCal_Tri& tri) {
	// if two triangles are on the same plane


	return true;
}