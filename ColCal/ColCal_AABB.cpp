#include "ColCal_AABB.h"

ColCal_AABB::ColCal_AABB() {
	this->Max[0] = this->Max[1] = this->Max[2] = 0;
	this->Min[0] = this->Min[1] = this->Min[2] = 0;
	this->idx = -1;
}
ColCal_AABB::ColCal_AABB(const ColCal_Tri& tri) {
	this->Max[0] = ColCal_Max(ColCal_Max(tri.Points[0].x, tri.Points[1].x), tri.Points[2].x);
	this->Max[1] = ColCal_Max(ColCal_Max(tri.Points[0].y, tri.Points[1].y), tri.Points[2].y);
	this->Max[2] = ColCal_Max(ColCal_Max(tri.Points[0].z, tri.Points[1].z), tri.Points[2].z);

	this->Min[0] = ColCal_Min(ColCal_Min(tri.Points[0].x, tri.Points[1].x), tri.Points[2].x);
	this->Min[1] = ColCal_Min(ColCal_Min(tri.Points[0].y, tri.Points[1].y), tri.Points[2].y);
	this->Min[2] = ColCal_Min(ColCal_Min(tri.Points[0].z, tri.Points[1].z), tri.Points[2].z);

	this->idx = tri.idx;
}
ColCal_AABB::ColCal_AABB(const ColCal_AABB& box) {
	this->Max[0] = box.Max[0];
	this->Max[1] = box.Max[1];
	this->Max[2] = box.Max[2];

	this->Min[0] = box.Min[0];
	this->Min[1] = box.Min[1];
	this->Min[2] = box.Min[2];

	this->idx = box.idx;
}
ColCal_AABB::ColCal_AABB(const ColCal_DataType x0, ColCal_DataType x1, ColCal_DataType y0, ColCal_DataType y1, ColCal_DataType z0, ColCal_DataType z1, int Idx) {
	this->Max[0] = x1;
	this->Max[1] = y1;
	this->Max[2] = z1;

	this->Min[0] = x0;
	this->Min[1] = y0;
	this->Min[2] = z0;

	this->idx = Idx;
}
ColCal_AABB& ColCal_AABB::operator=(const ColCal_AABB& box) {
	this->Max[0] = box.Max[0];
	this->Max[1] = box.Max[1];
	this->Max[2] = box.Max[2];

	this->Min[0] = box.Min[0];
	this->Min[1] = box.Min[1];
	this->Min[2] = box.Min[2];

	this->idx = box.idx;
	return *this;
}

void ColCal_AABB::Include(const ColCal_Tri& tri) {
	this->Min[0] = ColCal_Min(ColCal_Min(ColCal_Min(tri.Points[0].x, tri.Points[1].x), tri.Points[2].x), this->Min[0]);
	this->Max[0] = ColCal_Max(ColCal_Max(ColCal_Max(tri.Points[0].x, tri.Points[1].x), tri.Points[2].x), this->Max[0]);
	this->Min[1] = ColCal_Min(ColCal_Min(ColCal_Min(tri.Points[0].y, tri.Points[1].y), tri.Points[2].y), this->Min[1]);
	this->Max[1] = ColCal_Max(ColCal_Max(ColCal_Max(tri.Points[0].y, tri.Points[1].y), tri.Points[2].y), this->Max[1]);
	this->Min[2] = ColCal_Min(ColCal_Min(ColCal_Min(tri.Points[0].z, tri.Points[1].z), tri.Points[2].z), this->Min[1]);
	this->Max[2] = ColCal_Max(ColCal_Max(ColCal_Max(tri.Points[0].z, tri.Points[1].z), tri.Points[2].z), this->Max[2]);
}

ColCal_DataType ColCal_AABB::getSurfaceArea() {
	ColCal_DataType len_x = this->Max[0] - this->Min[0];
	ColCal_DataType len_y = this->Max[1] - this->Min[1];
	ColCal_DataType len_z = this->Max[2] - this->Min[2];
	return (len_x * len_y + len_x * len_z + len_y * len_z) * 2.0;
}

ColCal_DataType ColCal_AABB::getVolume() {
	ColCal_DataType len_x = this->Max[0] - this->Min[0];
	ColCal_DataType len_y = this->Max[1] - this->Min[1];
	ColCal_DataType len_z = this->Max[2] - this->Min[2];
	return len_x * len_y * len_z;
}

bool ColCal_AABB::collide(const ColCal_AABB& b) {
	for (int i = 0; i < 3; i++)
		if (b.Max[i] < this->Min[i] || b.Min[i]>this->Max[i])
			return false;
	return true;
}

bool ColCal_AABB::collide(const ColCal_AABB& b, ColCal_Mat4& M) {
	std::vector<ColCal_Vec4> pts = b.getPoints();

	ColCal_DataType max_, min_;
	// say *this is the base
	for (unsigned int i = 0; i < 3; i++) {
		max_ = ColCal_Min_Value;
		min_ = ColCal_Max_Value;
		for (unsigned int j = 0; j < pts.size(); i++) {
			ColCal_Vec4 p = M * pts[j];
			max_ = ColCal_Max(max_, p[i]);
			min_ = ColCal_Min(min_, p[i]);
		}
		if (max_< this->Min[i] || min_>this->Max[i])
			return false;
	}

	// say b is the base
	pts = this->getPoints();
	for (unsigned int i = 0; i < 3; i++) {
		max_ = ColCal_Min_Value;
		min_ = ColCal_Max_Value;
		for (unsigned int j = 0; j < pts.size(); i++) {
			ColCal_Vec4 p = M * pts[j];
			max_ = ColCal_Max(max_, p[i]);
			min_ = ColCal_Min(min_, p[i]);
		}
		if (max_< b.Min[i] || min_>b.Max[i])
			return false;
	}

	return true;
}

ColCal_DataType ColCal_AABB::getProximateDis(const ColCal_AABB& b, ColCal_Mat4& M) {
	ColCal_Vec3 p1(Min[0], Min[1], Min[2]);
	ColCal_Vec3 p2(Max[0], Max[1], Max[2]);
	ColCal_Vec3 center = (p1 + p2) * 0.5;
	ColCal_DataType r = (p2 - p1).length() * 0.5;

	ColCal_Vec4 p1__(b.Min[0], b.Min[1], b.Min[2]);
	ColCal_Vec4 p2__(b.Max[0], b.Max[1], b.Max[2]);
	p1__ = M * p1__;
	p2__ = M * p2__;

	ColCal_Vec3 p1_(p1__[0], p1__[1], p1__[2]);
	ColCal_Vec3 p2_(p2__[0], p2__[1], p2__[2]);
	ColCal_Vec3 center_ = (p2_ + p1_) * 0.5;
	ColCal_DataType r_ = (p2_ - p1_).length() * 0.5;

	return (center_ - center).length() - r - r_;
}

bool ColCal_AABB::collide_axis(const ColCal_AABB b, const int axis) {
	if (b.Max[axis] < this->Min[axis] || b.Min[axis]>this->Max[axis])
		return false;
	return true;
}

ColCal_DataType ColCal_AABB::getMax(int x)const {
	return this->Max[x];
}
ColCal_DataType ColCal_AABB::getMin(int x)const {
	return this->Min[x];
}
unsigned int ColCal_AABB::getIdx()const {
	return this->idx;
}

std::vector<ColCal_Vec4> ColCal_AABB::getPoints()const {
	std::vector<ColCal_Vec4> ret;
	ret.push_back(ColCal_Vec4(Min[0], Min[1], Min[2]));
	ret.push_back(ColCal_Vec4(Min[0], Min[1], Max[2]));
	ret.push_back(ColCal_Vec4(Min[0], Max[1], Min[2]));
	ret.push_back(ColCal_Vec4(Min[0], Max[1], Max[2]));
	ret.push_back(ColCal_Vec4(Max[0], Min[1], Min[2]));
	ret.push_back(ColCal_Vec4(Max[0], Min[1], Max[2]));
	ret.push_back(ColCal_Vec4(Max[0], Max[1], Min[2]));
	ret.push_back(ColCal_Vec4(Max[0], Max[1], Max[2]));
	return ret;
}