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
	bool ColCal_Point_Inside_Triangle(const ColCal_Point& p) const;
	bool ColCal_Collision(const ColCal_Tri& tri, bool considerParallel = false);

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

inline bool TriCompare_X(const ColCal_Tri*& A, const ColCal_Tri*& B) {
	return A->Points[0].x + A->Points[1].x + A->Points[2].x < B->Points[0].x + B->Points[1].x + B->Points[2].x;
};

inline bool TriCompare_Y(const ColCal_Tri*& A, const ColCal_Tri*& B) {
	return  A->Points[0].y + A->Points[1].y + A->Points[2].y < B->Points[0].y + B->Points[1].y + B->Points[2].y;
};

inline bool TriCompare_Z(const ColCal_Tri*& A, const ColCal_Tri*& B) {
	return A->Points[0].z + A->Points[1].z + A->Points[2].z < B->Points[0].z + B->Points[1].z + B->Points[2].z;
};

#endif // !COLCAL_TRI_H