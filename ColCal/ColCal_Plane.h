#ifndef COLCAL_PLANE_H
#define COLCAL_PLANE_H

#include "ColCal.h"
#include "ColCal_Point.h"
#include "ColCal_Math.h"

class ColCal_Plane
{
public:
	ColCal_Plane();


	//****************COlCal_Point waits for amending!***********//
	ColCal_Point point;
	//****************COlCal_Point waits for amending!***********//

	ColCal_Vec3 arrow;
};

#endif // !COLCAL_PLANE_H
