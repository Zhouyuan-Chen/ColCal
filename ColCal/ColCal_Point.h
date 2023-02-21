#ifndef COLCAL_POINT_H
#define COLCAL_POINT_H

#include "ColCal.h"

class ColCal_Point {
public:
	ColCal_Point();
	ColCal_Point(const ColCal_DataType& X, const ColCal_DataType& Y, const ColCal_DataType& Z);
	ColCal_Point(const ColCal_Point& point);
	ColCal_Point& operator=(const ColCal_Point& point);
	ColCal_DataType& operator[](const int x);
	ColCal_DataType x, y, z;
};

inline bool PointCompare_X(ColCal_Point*& A, ColCal_Point*& B) {
	return A->x < B->x;
};

inline bool PointCompare_Y(ColCal_Point*& A, ColCal_Point*& B) {
	return A->y < B->y;
};

inline bool PointCompare_Z(ColCal_Point*& A, ColCal_Point*& B) {
	return A->z < B->z;
};

#endif // !POINT_H