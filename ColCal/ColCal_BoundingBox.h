#ifndef COLCAL_BOUNDINGBOX_H
#define COLCAL_BOUNDINGBOX_H

#include "ColCal_OBB.h"
#include "ColCal_AABB.h"

// Adapter Mode
enum class BoxType
{
	OBB, AABB /* , HLBVH */
};

class ColCal_BoundingBox {
public:
	ColCal_BoundingBox();

	~ColCal_BoundingBox();

	ColCal_BoundingBox(const ColCal_BoundingBox& box);

	// only for AABB
	ColCal_BoundingBox(const ColCal_Tri& tri);
	// only for AABB
	ColCal_BoundingBox(const ColCal_DataType x0, ColCal_DataType x1, ColCal_DataType y0, ColCal_DataType y1, ColCal_DataType z0, ColCal_DataType z1, int Idx = -1);
	// only for AABB box
	void Include(const ColCal_Tri& tri);
	// only for AABB box
	bool collide_axis(const ColCal_BoundingBox b, const int axis = 0);

	// only for OBB
	ColCal_BoundingBox(const std::vector<ColCal_Point*>& pts, unsigned int index = -1);
	// only for OBB
	ColCal_BoundingBox(const std::vector<ColCal_Tri*>& tris, const std::vector<ColCal_Point*>& pts, unsigned int index = -1);
	
	ColCal_BoundingBox& operator=(const ColCal_BoundingBox& box);

	bool collide(const ColCal_BoundingBox& box);

	ColCal_DataType getSurfaceArea();

	ColCal_DataType getVolume();

	ColCal_DataType getMax(int x)const;

	ColCal_DataType getMin(int x)const;

	unsigned int getIdx()const;

private:
	BoxType type;
	ColCal_AABB* aabb;
	ColCal_OBB* obb;
};

#endif // !COLCAL_BOUNDINGBOX