#ifndef COLCAL_TRI_H
#define COLCAL_TRI_H

#include "ColCal.h"
#include "ColCal_Point.h"
#include "ColCal_Math.h"

class ColCal_Tri {
public:
	ColCal_Tri();

	ColCal_Tri(const ColCal_Point p1, const ColCal_Point p2, const ColCal_Point p3, int Idx);
	ColCal_Tri(const ColCal_Tri& tri);
	ColCal_Tri& operator=(const ColCal_Tri& tri);

	bool ColCal_Collision(const ColCal_Tri& tri);

	int idx;
	ColCal_Point Points[3];
};

class ColCal_Tri_Pair {
public:
	ColCal_Tri_Pair(ColCal_Tri t1, ColCal_Tri t2) {
		tri1 = t1;
		tri2 = t2;
	}
	ColCal_Tri tri1;
	ColCal_Tri tri2;
};

#endif // !COLCAL_TRI_H