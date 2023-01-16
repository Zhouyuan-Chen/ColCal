#ifndef COLCAL_TRI_H

#include "ColCal.hpp"
#include "ColCal_Point.hpp"

class ColCal_Tri {
public:
	ColCal_Tri() {
		this->Point[0] = ColCal_Point(0, 0, 0);
		this->Point[1] = ColCal_Point(0, 0, 0);
		this->Point[2] = ColCal_Point(0, 0, 0);
		this->idx = -1;
	}

	ColCal_Tri(ColCal_Point p1, ColCal_Point p2, ColCal_Point p3, int Idx) {
		this->Point[0] = p1;
		this->Point[1] = p2;
		this->Point[2] = p3;
		this->idx = Idx;
	}

	bool ColCal_Collision(const ColCal_Tri& tri) {
		// ...

		return true;
	}

	int idx;
	ColCal_Point Point[3];
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