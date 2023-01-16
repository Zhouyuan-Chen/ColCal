#ifndef COLCAL_H
#define COLCAL_H
////////////////////////////////////////////////////////////////
// COLLISION CALCULATION LIBRARY (ColCal)
////////////////////////////////////////////////////////////////
// @author Zhouyuan Chen
// @date 2023/1/16
// /////////////////////////////////////////////////////////////
// 
// In this lib, I implemented some collision detect methods, you
// can use this lib to simulate normal geometry or phicial collision.
//
// NOTE: this lib uses right-hand rule	
// 
//	   |y
//	   |   / z
//	   |  /
//	   | /
//	   |/_ _ _ _ _ x
// 
/////////////////////////////////////////////////////////////////

#include <vector>
#define ColCal_Max(a,b) ((a) > (b) ? (a) : (b))
#define ColCal_Min(a,b) ((a) < (b) ? (a) : (b))

struct ColCal_Point {
	ColCal_Point() {
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}
	ColCal_Point(float X,float Y,float Z) {
		this->x = X;
		this->y = Y;
		this->z = Z;
	}
	float x, y, z;
};

struct ColCal_Tri {
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
	int idx;
	ColCal_Point Point[3];
};

struct ColCal_Pair {
	ColCal_Tri tri1;
	ColCal_Tri tri2;
};

struct ColCal_Box {
	ColCal_Box(ColCal_Tri tri) {
		this->Max[0] = ColCal_Max(ColCal_Max(tri.Point[0].x, tri.Point[1].x), tri.Point[2].x);
		this->Max[1] = ColCal_Max(ColCal_Max(tri.Point[0].y, tri.Point[1].y), tri.Point[2].y);
		this->Max[2] = ColCal_Max(ColCal_Max(tri.Point[0].z, tri.Point[1].z), tri.Point[2].z);

		this->Min[0] = ColCal_Min(ColCal_Min(tri.Point[0].x, tri.Point[1].x), tri.Point[2].x);
		this->Min[1] = ColCal_Min(ColCal_Min(tri.Point[0].y, tri.Point[1].y), tri.Point[2].y);
		this->Min[2] = ColCal_Min(ColCal_Min(tri.Point[0].z, tri.Point[1].z), tri.Point[2].z);

		this->idx = tri.idx;
	}
	int idx;
	float Max[3];
	float Min[3];
};

/////////////////////////////////////////////////////////////////
//  DIAGRAM FOR BOX COLLISION
/////////////////////////////////////////////////////////////////
//      	_ _ _ _ _ _ 
//		   /		   /|
//		  /			  / |<---behind
//		 /	top		 /  |
//		/_ _ _ _ _ _/   | 
//left->|			| <-|- - - - right
//		|			|   |
//		|	front	|  / 
//		|			| /
//		|_ _ _ _ _ _|/
//			^
//			|_ bottom
// 
// 	   |y
//	   |   / z
//	   |  /
//	   | /
//	   |/_ _ _ _ _ x
// 
/////////////////////////////////////////////////////////////////

bool ColCal_Box_Collision(ColCal_Box a, ColCal_Box b) {
	// box a's left-bottom-front point is inside b box
	if (a.Min[0] > b.Min[0] && a.Min[0] < b.Max[0]		// x
		&& a.Min[1] > b.Min[1] && a.Min[1] < b.Max[1]	// y
		&& a.Min[2] > b.Min[2] && a.Min[2] < b.Max[2]	// z
		)
		return true;

	// box a's right-bottom-front point is inside b box
	if (a.Max[0] > b.Min[0] && a.Max[0] < b.Max[0]		// x
		&& a.Min[1] > b.Min[1] && a.Min[1] < b.Max[1]	// y
		&& a.Min[2] > b.Min[2] && a.Min[2] < b.Max[2]	// z
		)
		return true;

	// box a's left-top-front point is inside b box
	if (a.Min[0] > b.Min[0] && a.Min[0] < b.Max[0]		// x
		&& a.Max[1] > b.Min[1] && a.Max[1] < b.Max[1]	// y
		&& a.Min[2] > b.Min[2] && a.Min[2] < b.Max[2]	// z
		)
		return true;

	// box a's right-top-front point is inside b box
	if (a.Max[0] > b.Min[0] && a.Max[0] < b.Max[0]		// x
		&& a.Max[1] > b.Min[1] && a.Max[1] < b.Max[1]	// y
		&& a.Min[2] > b.Min[2] && a.Min[2] < b.Max[2]	// z
		)
		return true;

	// box a's left-bottom-behind point is inside b box
	if (a.Min[0] > b.Min[0] && a.Min[0] < b.Max[0]		// x
		&& a.Min[1] > b.Min[1] && a.Min[1] < b.Max[1]	// y
		&& a.Max[2] > b.Min[2] && a.Max[2] < b.Max[2]	// z
		)
		return true;

	// box a's right-bottom-behind point is inside b box
	if (a.Max[0] > b.Min[0] && a.Max[0] < b.Max[0]		// x
		&& a.Min[1] > b.Min[1] && a.Min[1] < b.Max[1]	// y
		&& a.Max[2] > b.Min[2] && a.Max[2] < b.Max[2]	// z
		)
		return true;

	// box a's left-top-behind point is inside b box
	if (a.Min[0] > b.Min[0] && a.Min[0] < b.Max[0]		// x
		&& a.Max[1] > b.Min[1] && a.Max[1] < b.Max[1]	// y
		&& a.Max[2] > b.Min[2] && a.Max[2] < b.Max[2]	// z
		)
		return true;

	// box a's right-top-behind point is inside b box
	if (a.Max[0] > b.Min[0] && a.Max[0] < b.Max[0]		// x
		&& a.Max[1] > b.Min[1] && a.Max[1] < b.Max[1]	// y
		&& a.Max[2] > b.Min[2] && a.Max[2] < b.Max[2]	// z
		)
		return true;
}

class ColCal_Objects {
public:
	ColCal_Objects() {
		this->TriArray.clear();
		this->BoxArray.clear();
		this->idx = -1;
	}
	ColCal_Objects(std::vector<ColCal_Tri> tris, int Idx) {
		for (ColCal_Tri tri : tris) {
			this->TriArray.push_back(tri);
			ColCal_Box box = ColCal_Box(tri);
			this->BoxArray.push_back(box);
		}
		this->idx = Idx;
	}
	int idx;
	std::vector<ColCal_Tri> TriArray;
	std::vector<ColCal_Box> BoxArray;
};

class ColCal {

};

#endif // !COLCAL_H
