#include "ColCal_Box.h"

ColCal_Box::ColCal_Box() {
	this->Max[0] = this->Max[1] = this->Max[2] = 0;
	this->Min[0] = this->Min[1] = this->Min[2] = 0;
	this->idx = -1;
}
ColCal_Box::ColCal_Box(const ColCal_Tri& tri) {
	this->Max[0] = ColCal_Max(ColCal_Max(tri.Points[0].x, tri.Points[1].x), tri.Points[2].x);
	this->Max[1] = ColCal_Max(ColCal_Max(tri.Points[0].y, tri.Points[1].y), tri.Points[2].y);
	this->Max[2] = ColCal_Max(ColCal_Max(tri.Points[0].z, tri.Points[1].z), tri.Points[2].z);

	this->Min[0] = ColCal_Min(ColCal_Min(tri.Points[0].x, tri.Points[1].x), tri.Points[2].x);
	this->Min[1] = ColCal_Min(ColCal_Min(tri.Points[0].y, tri.Points[1].y), tri.Points[2].y);
	this->Min[2] = ColCal_Min(ColCal_Min(tri.Points[0].z, tri.Points[1].z), tri.Points[2].z);

	this->idx = tri.idx;
}
ColCal_Box::ColCal_Box(const ColCal_Box& box) {
	this->Max[0] = box.Max[0];
	this->Max[1] = box.Max[1];
	this->Max[2] = box.Max[2];

	this->Min[0] = box.Min[0];
	this->Min[1] = box.Min[1];
	this->Min[2] = box.Min[2];

	this->idx = box.idx;
}
ColCal_Box& ColCal_Box::operator=(const ColCal_Box& box) {
	this->Max[0] = box.Max[0];
	this->Max[1] = box.Max[1];
	this->Max[2] = box.Max[2];

	this->Min[0] = box.Min[0];
	this->Min[1] = box.Min[1];
	this->Min[2] = box.Min[2];

	this->idx = box.idx;
	return *this;
}

bool ColCal_Box::ColCal_Collision(const ColCal_Box b) {
	for (int i = 0; i < 3; i++)
		if (b.Max[i] < this->Min[i] || b.Min[i]>this->Max[i])
			return false;
	return true;
}

bool ColCal_Box::ColCal_Collision_Axis(const ColCal_Box b, const int axis) {
	if (b.Max[axis] < this->Min[axis] || b.Min[axis]>this->Max[axis])
		return false;
	return true;
}