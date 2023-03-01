#ifndef COlCAL_OBB
#define COLCAL_OBB

#include "ColCal.h"
#include "ColCal_Tri.h"

class ColCal_OBB {
public:
	ColCal_OBB();
	~ColCal_OBB();
	ColCal_OBB(const std::vector<ColCal_Point*>& pts);
	ColCal_OBB(const std::vector<ColCal_Tri*>& tris, const std::vector<ColCal_Point*>& pts);
	void build(const std::vector<ColCal_Point*>& pts); // for points
	void build(const std::vector<ColCal_Tri*>& tris, const std::vector<ColCal_Point*>& pts); // for triangles
	// Separating Axis Theorem(SAT for short)
	bool collide(const ColCal_OBB& obb);
	void update();
	ColCal_Point* getBoxPoints();
private:
	void Build_with_Covariance(const ColCal_Mat4& cov, const std::vector<ColCal_Point*>& pts);
	ColCal_Mat4 rotate;
	ColCal_Vec3 pos; // also for transformation
	ColCal_Vec3 extension;
	ColCal_Point* points;
};

#endif // COlCAL_OBB
