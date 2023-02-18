#ifndef COLCAL_BOX_H
#define COLCAL_BOX_H

#include "ColCal.h"
#include "ColCal_Tri.h"

class ColCal_Box {
public:
	ColCal_Box();
	ColCal_Box(const ColCal_Tri& tri);
	ColCal_Box(const ColCal_Box& box);
	ColCal_Box(const ColCal_DataType x0, ColCal_DataType x1, ColCal_DataType y0, ColCal_DataType y1, ColCal_DataType z0, ColCal_DataType z1, int Idx = -1);
	ColCal_Box& operator=(const ColCal_Box& box);

	// Note: this function can not be used for initiating a new ColCal_Box instance
	void Include(const ColCal_Tri& tri);

	//void Union();

	bool ColCal_Collision(const ColCal_Box b);
	bool ColCal_Collision_Axis(const ColCal_Box b, const int axis = 0);

	int idx;
	ColCal_DataType Max[3];
	ColCal_DataType Min[3];
};

#endif