#ifndef COLCAL_POINT_H

struct ColCal_Point {
	ColCal_Point() {
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}
	ColCal_Point(float X, float Y, float Z) {
		this->x = X;
		this->y = Y;
		this->z = Z;
	}
	float x, y, z;
};

#endif // !POINT_H