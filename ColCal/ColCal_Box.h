#ifndef COLCAL_BOX_H
#define COLCAL_BOX_H

#include "ColCal.h"
#include "ColCal_Tri.h"

class ColCal_Box {
public:
	ColCal_Box();
	ColCal_Box(const ColCal_Tri& tri);
	ColCal_Box(const ColCal_Box& box);
	ColCal_Box& operator=(const ColCal_Box& box);

	bool ColCal_Collision(const ColCal_Box b);
	bool ColCal_Collision_Axis(const ColCal_Box b, const int axis = 0);

	int idx;
	float Max[3];
	float Min[3];
};

#endif