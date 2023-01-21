#include "ColCal_Plane.h"

ColCal_Plane::ColCal_Plane() {
	this->arrow = ColCal_Vec3();
}
ColCal_Plane::ColCal_Plane(const ColCal_Point& p, const ColCal_Vec3& arr) {
	this->arrow = arr;
	this->point = p;
}
ColCal_Plane::ColCal_Plane(const ColCal_Plane& p) {
	this->arrow = p.arrow;
	this->point = p.point;
}
ColCal_Plane::ColCal_Plane(const ColCal_Tri& tri) {
	ColCal_Vec3 p1 = ColCal_Vec3(tri.Points[0].x, tri.Points[0].y, tri.Points[0].z);
	ColCal_Vec3 p2 = ColCal_Vec3(tri.Points[1].x, tri.Points[1].y, tri.Points[1].z);
	ColCal_Vec3 p3 = ColCal_Vec3(tri.Points[2].x, tri.Points[2].y, tri.Points[2].z);

	this->point = tri.Points[0];
	ColCal_Vec3 vec1 = p3 - p1;
	ColCal_Vec3 vec2 = p2 - p1;
	this->arrow = vec1 ^ vec2;
}
ColCal_Plane& ColCal_Plane::operator=(const ColCal_Plane& p) {
	this->point = p.point;
	this->arrow = p.arrow;
	return *this;
}