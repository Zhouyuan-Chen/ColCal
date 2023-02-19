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
ColCal_Box::ColCal_Box(const ColCal_DataType x0, ColCal_DataType x1, ColCal_DataType y0, ColCal_DataType y1, ColCal_DataType z0, ColCal_DataType z1, int Idx) {
	this->Max[0] = x1;
	this->Max[1] = y1;
	this->Max[2] = z1;

	this->Min[0] = x0;
	this->Min[1] = y0;
	this->Min[2] = z0;

	this->idx = Idx;
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

void ColCal_Box::Include(const ColCal_Tri& tri) {
	this->Min[0] = ColCal_Min(ColCal_Min(ColCal_Min(tri.Points[0].x, tri.Points[1].x), tri.Points[2].x), this->Min[0]);
	this->Max[0] = ColCal_Max(ColCal_Max(ColCal_Max(tri.Points[0].x, tri.Points[1].x), tri.Points[2].x), this->Max[0]);
	this->Min[1] = ColCal_Min(ColCal_Min(ColCal_Min(tri.Points[0].y, tri.Points[1].y), tri.Points[2].y), this->Min[1]);
	this->Max[1] = ColCal_Max(ColCal_Max(ColCal_Max(tri.Points[0].y, tri.Points[1].y), tri.Points[2].y), this->Max[1]);
	this->Min[2] = ColCal_Min(ColCal_Min(ColCal_Min(tri.Points[0].z, tri.Points[1].z), tri.Points[2].z), this->Min[1]);
	this->Max[2] = ColCal_Max(ColCal_Max(ColCal_Max(tri.Points[0].z, tri.Points[1].z), tri.Points[2].z), this->Max[2]);
}

ColCal_DataType ColCal_Box::getSurfaceArea() {
	ColCal_DataType len_x = this->Max[0] - this->Min[0];
	ColCal_DataType len_y = this->Max[1] - this->Min[1];
	ColCal_DataType len_z = this->Max[2] - this->Min[2];
	return (len_x * len_y + len_x * len_z + len_y * len_z) * 2.0;
}

ColCal_DataType ColCal_Box::getVolume() {
	ColCal_DataType len_x = this->Max[0] - this->Min[0];
	ColCal_DataType len_y = this->Max[1] - this->Min[1];
	ColCal_DataType len_z = this->Max[2] - this->Min[2];
	return len_x * len_y * len_z;
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