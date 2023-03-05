#ifndef COLCAL_RESULT_H

#include "ColCal_BVH.h"

class ColCal_Result_BVH {
public:
	ColCal_Result_BVH();
	bool CalculateCollide(ColCal_BVH& A, ColCal_Mat4& R1, ColCal_Vec4& T1, ColCal_BVH& B, ColCal_Mat4& R2, ColCal_Vec4& T2);
	void CalculateCollide(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M);
	void CalculatePairs(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M);
	void CalculatePairsDistance(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M);
	ColCal_DataType CalculateDistance(ColCal_BVH& A, ColCal_Mat4& R1, ColCal_Vec4& T1, ColCal_BVH& B, ColCal_Mat4& R2, ColCal_Vec4& T2);
	void CalculateDistance(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M);
	unsigned int CollisionNum();
	ColCal_DataType getDistance();
private:
	std::vector<std::pair<unsigned int, unsigned int>> result;
	ColCal_DataType distance;
	ColCal_BVH* BVH_A;
	ColCal_BVH* BVH_B;
};

#endif // !COLCAL_RESULT_H
