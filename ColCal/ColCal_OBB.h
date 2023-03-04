#ifndef COlCAL_OBB_H
#define COLCAL_OBB_H

#include "ColCal.h"
#include "ColCal_Tri.h"

class ColCal_OBB {
public:
	ColCal_OBB();
	~ColCal_OBB();
	ColCal_OBB(const std::vector<ColCal_Point*>& pts, unsigned int index = 0);
	ColCal_OBB(const std::vector<ColCal_Tri*>& tris, const std::vector<ColCal_Point*>& pts, unsigned int index = 0);
	void build(const std::vector<ColCal_Point*>& pts); // for points
	void build(const std::vector<ColCal_Tri*>& tris, const std::vector<ColCal_Point*>& pts); // for triangles
	// Separating Axis Theorem(SAT for short)
	bool collide(const ColCal_OBB& obb);
	ColCal_DataType getSurfaceArea();
	ColCal_DataType getVolume();
	void update();
	ColCal_Point* getBoxPoints();
	ColCal_DataType getMax(int x)const;
	ColCal_DataType getMin(int x)const;
	unsigned int getIdx()const;
private:
	void Build_with_Covariance(const ColCal_Mat4& cov, const std::vector<ColCal_Point*>& pts);
	ColCal_Mat4 rotate;
	ColCal_Vec3 pos; // also for transformation
	ColCal_Vec3 extension;
	ColCal_Point* points;
	unsigned int idx;
};

#endif // COlCAL_OBB
