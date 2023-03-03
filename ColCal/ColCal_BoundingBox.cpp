#include "ColCal_BoundingBox.h"

ColCal_BoundingBox::ColCal_BoundingBox() {
	aabb = nullptr;
	obb = nullptr;
	type = BoxType::AABB;
}
ColCal_BoundingBox::~ColCal_BoundingBox() {
	if (aabb)
		delete aabb;
	aabb = nullptr;
	if (obb)
		delete obb;
	obb = nullptr;
}

ColCal_BoundingBox::ColCal_BoundingBox(const ColCal_BoundingBox& box) {
	this->aabb = nullptr;
	this->obb = nullptr;
	if (box.aabb)
		this->aabb = new ColCal_AABB(*(box.aabb));
	if (box.obb)
		this->obb = new ColCal_OBB(*(box.obb));
	this->type = box.type;
}

// only for AABB
ColCal_BoundingBox::ColCal_BoundingBox(const ColCal_Tri& tri) {
	this->aabb = nullptr;
	this->obb = nullptr;

	this->type = BoxType::AABB;
	this->aabb = new ColCal_AABB(tri);
}
// only for AABB
ColCal_BoundingBox::ColCal_BoundingBox(const ColCal_DataType x0, ColCal_DataType x1, ColCal_DataType y0, ColCal_DataType y1, ColCal_DataType z0, ColCal_DataType z1, int Idx) {
	this->aabb = nullptr;
	this->obb = nullptr;

	this->type = BoxType::AABB;
	this->aabb = new ColCal_AABB(x0, x1, y0, y1, z0, z1, Idx);
}
// only for OBB
ColCal_BoundingBox::ColCal_BoundingBox(const std::vector<ColCal_Point*>& pts, unsigned int index) {
	this->aabb = nullptr;
	this->obb = nullptr;

	this->type = BoxType::OBB;
	this->obb = new ColCal_OBB(pts, index);
}
// only for OBB
ColCal_BoundingBox::ColCal_BoundingBox(const std::vector<ColCal_Tri*>& tris, const std::vector<ColCal_Point*>& pts, unsigned int index) {
	this->aabb = nullptr;
	this->obb = nullptr;

	this->type = BoxType::OBB;
	this->obb = new ColCal_OBB(tris, pts, index);
}

ColCal_BoundingBox& ColCal_BoundingBox::operator=(const ColCal_BoundingBox& box) {
	if (aabb)
		delete aabb;
	aabb = nullptr;
	if (obb)
		delete obb;
	obb = nullptr;

	if (type == BoxType::AABB && box.aabb) {
		this->aabb = new ColCal_AABB(*(box.aabb));
	}
	if (type == BoxType::OBB && box.obb) {
		this->obb = new ColCal_OBB(*(box.obb));
	}

	return *this;
}

bool ColCal_BoundingBox::collide(const ColCal_BoundingBox& box) {
	if (type == BoxType::AABB && this->aabb && box.aabb)
		return aabb->collide(*(box.aabb));
	if (type == BoxType::OBB && this->obb && box.obb)
		return obb->collide(*(box.obb));
	return false;
}

// only for AABB box
void ColCal_BoundingBox::Include(const ColCal_Tri& tri) {
	if (aabb)
		this->aabb->Include(tri);
}
// only for AABB box
bool ColCal_BoundingBox::collide_axis(const ColCal_BoundingBox b, const int axis) {
	if (aabb)
		return this->aabb->collide_axis(*(b.aabb), axis);
	return false;
}


ColCal_DataType ColCal_BoundingBox::getSurfaceArea() {
	if (type == BoxType::AABB && aabb)
		return aabb->getSurfaceArea();
	if (type == BoxType::OBB && obb)
		return obb->getSurfaceArea();
	return 0;
}

ColCal_DataType ColCal_BoundingBox::getVolume() {
	if (type == BoxType::AABB && aabb)
		return aabb->getVolume();
	if (type == BoxType::OBB && obb)
		return obb->getVolume();
	return 0;
}

ColCal_DataType ColCal_BoundingBox::getMax(int x)const {
	if (type == BoxType::AABB && aabb)
		return aabb->getMax(x);
	if (type == BoxType::OBB && obb)
		return obb->getMax(x);
	return 0;
}

ColCal_DataType ColCal_BoundingBox::getMin(int x)const {
	if (type == BoxType::AABB && aabb)
		return aabb->getMin(x);
	if (type == BoxType::OBB && obb)
		return obb->getMin(x);
	return 0;
}
unsigned int ColCal_BoundingBox::getIdx()const {
	if (type == BoxType::AABB && aabb)
		return aabb->getIdx();
	if (type == BoxType::OBB && obb)
		return obb->getIdx();
	return 0;
}
