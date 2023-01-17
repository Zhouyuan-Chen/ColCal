#include "ColCal_Box.h"

/////////////////////////////////////////////////////////////////
//  DIAGRAM FOR BOX COLLISION
/////////////////////////////////////////////////////////////////
//      	 
//         _ _ _ _ _ _		   
//        /           /|<---behind
//       /    top    / |
//      /_ _ _ _ _ _/  | 
//left->|           |<-|- - - - right
//      |           |  |
//      |   front   |  / 
//      |           | /
//      |_ _ _ _ _ _|/
//          ^
//          |
//        bottom
// 
//     |y
//	   |   / z
//	   |  /
//	   | /
//	   |/_ _ _ _ _ x
// 
/////////////////////////////////////////////////////////////////

ColCal_Box::ColCal_Box() {
	this->Max[0] = this->Max[1] = this->Max[2] = 0;
	this->Min[0] = this->Min[1] = this->Min[2] = 0;
	this->idx = -1;
}
ColCal_Box::ColCal_Box(const ColCal_Tri& tri) {
	this->Max[0] = ColCal_Max(ColCal_Max(tri.Point[0].x, tri.Point[1].x), tri.Point[2].x);
	this->Max[1] = ColCal_Max(ColCal_Max(tri.Point[0].y, tri.Point[1].y), tri.Point[2].y);
	this->Max[2] = ColCal_Max(ColCal_Max(tri.Point[0].z, tri.Point[1].z), tri.Point[2].z);

	this->Min[0] = ColCal_Min(ColCal_Min(tri.Point[0].x, tri.Point[1].x), tri.Point[2].x);
	this->Min[1] = ColCal_Min(ColCal_Min(tri.Point[0].y, tri.Point[1].y), tri.Point[2].y);
	this->Min[2] = ColCal_Min(ColCal_Min(tri.Point[0].z, tri.Point[1].z), tri.Point[2].z);

	this->idx = tri.idx;
}
ColCal_Box::ColCal_Box(const ColCal_Box& box) {
	this->Max[0] = box.Max[0];
	this->Max[1] = box.Max[1];
	this->Max[2] = box.Max[2];

	this->Min[0] = box.Min[0];
	this->Min[1] = box.Min[1];
	this->Min[2] = box.Min[2];

	this->idx = box.idx;
}
ColCal_Box& ColCal_Box::operator=(const ColCal_Box& box) {
	this->Max[0] = box.Max[0];
	this->Max[1] = box.Max[1];
	this->Max[2] = box.Max[2];

	this->Min[0] = box.Min[0];
	this->Min[1] = box.Min[1];
	this->Min[2] = box.Min[2];

	this->idx = box.idx;
	return *this;
}

bool ColCal_Box::ColCal_Collision(const ColCal_Box b) {
	// box a's left-bottom-front point is inside b box
	if (this->Min[0] > b.Min[0] && this->Min[0] < b.Max[0]		// x
		&& this->Min[1] > b.Min[1] && this->Min[1] < b.Max[1]	// y
		&& this->Min[2] > b.Min[2] && this->Min[2] < b.Max[2]	// z
		)
		return true;

	// box a's right-bottom-front point is inside b box
	if (this->Max[0] > b.Min[0] && this->Max[0] < b.Max[0]		// x
		&& this->Min[1] > b.Min[1] && this->Min[1] < b.Max[1]	// y
		&& this->Min[2] > b.Min[2] && this->Min[2] < b.Max[2]	// z
		)
		return true;

	// box a's left-top-front point is inside b box
	if (this->Min[0] > b.Min[0] && this->Min[0] < b.Max[0]		// x
		&& this->Max[1] > b.Min[1] && this->Max[1] < b.Max[1]	// y
		&& this->Min[2] > b.Min[2] && this->Min[2] < b.Max[2]	// z
		)
		return true;

	// box a's right-top-front point is inside b box
	if (this->Max[0] > b.Min[0] && this->Max[0] < b.Max[0]		// x
		&& this->Max[1] > b.Min[1] && this->Max[1] < b.Max[1]	// y
		&& this->Min[2] > b.Min[2] && this->Min[2] < b.Max[2]	// z
		)
		return true;

	// box a's left-bottom-behind point is inside b box
	if (this->Min[0] > b.Min[0] && this->Min[0] < b.Max[0]		// x
		&& this->Min[1] > b.Min[1] && this->Min[1] < b.Max[1]	// y
		&& this->Max[2] > b.Min[2] && this->Max[2] < b.Max[2]	// z
		)
		return true;

	// box a's right-bottom-behind point is inside b box
	if (this->Max[0] > b.Min[0] && this->Max[0] < b.Max[0]		// x
		&& this->Min[1] > b.Min[1] && this->Min[1] < b.Max[1]	// y
		&& this->Max[2] > b.Min[2] && this->Max[2] < b.Max[2]	// z
		)
		return true;

	// box a's left-top-behind point is inside b box
	if (this->Min[0] > b.Min[0] && this->Min[0] < b.Max[0]		// x
		&& this->Max[1] > b.Min[1] && this->Max[1] < b.Max[1]	// y
		&& this->Max[2] > b.Min[2] && this->Max[2] < b.Max[2]	// z
		)
		return true;

	// box a's right-top-behind point is inside b box
	if (this->Max[0] > b.Min[0] && this->Max[0] < b.Max[0]		// x
		&& this->Max[1] > b.Min[1] && this->Max[1] < b.Max[1]	// y
		&& this->Max[2] > b.Min[2] && this->Max[2] < b.Max[2]	// z
		)
		return true;
	return false;
}

bool ColCal_Box::ColCal_Collision_Axis(const ColCal_Box B, const int axis) {
	if (this->Min[axis] > B.Min[axis] && this->Min[axis] < B.Max[axis])
		return true;
	if (this->Max[axis] > B.Min[axis] && this->Max[axis] < B.Max[axis])
		return true;
	return false;
}