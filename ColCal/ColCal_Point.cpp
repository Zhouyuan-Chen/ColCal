#include "ColCal_Point.h"

ColCal_Point::ColCal_Point() {
	this->x = 0;
	this->y = 0;
	this->z = 0;
}
ColCal_Point::ColCal_Point(const ColCal_DataType& X, const ColCal_DataType& Y, const ColCal_DataType& Z) {
	this->x = X;
	this->y = Y;
	this->z = Z;
}
ColCal_Point::ColCal_Point(const ColCal_Point& point) {
	this->x = point.x;
	this->y = point.y;
	this->z = point.z;
}
ColCal_Point& ColCal_Point::operator=(const ColCal_Point& point) {
	this->x = point.x;
	this->y = point.y;
	this->z = point.z;
	return *this;
}
ColCal_DataType& ColCal_Point::operator[](const int x) {
	if (x == 0)
		return this->x;
	if (x == 1)
		return this->y;
	if (x == 2)
		return this->z;
	exit(1);
}