#ifndef COLCAL_PLANE_H
#define COLCAL_PLANE_H

#include "ColCal.h"
#include "ColCal_Point.h"
#include "ColCal_Math.h"
#include "ColCal_Tri.h"

class ColCal_Plane
{
public:
	ColCal_Plane();
	ColCal_Plane(const ColCal_Point&,const ColCal_Vec3&);
	ColCal_Plane(const ColCal_Plane&);
	ColCal_Plane(const ColCal_Tri&);
	ColCal_Plane& operator=(const ColCal_Plane&);

	ColCal_Point point;
	ColCal_Vec3 arrow;
};

#endif // !COLCAL_PLANE_H
