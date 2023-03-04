#ifndef COLCAL_RESULT_H

#include "ColCal_BVH.h"

class ColCal_Result_BVH {
public:
	bool Collide(ColCal_BVH& A, ColCal_Mat4& R1, ColCal_Vec4& T1, ColCal_BVH& B, ColCal_Mat4& R2, ColCal_Vec4& T2);
	void Collide(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M);
	void CalculatePairs(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M);
	ColCal_DataType Distance(ColCal_BVH& A, ColCal_Mat4& R1, ColCal_Vec4& T1, ColCal_BVH& B, ColCal_Mat4& R2, ColCal_Vec4& T2);
	void Distance(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M);
	unsigned int CollisionNum();
private:
	std::vector<std::pair<unsigned int, unsigned int>> result;
	ColCal_BVH* BVH_A;
	ColCal_BVH* BVH_B;
};

#endif // !COLCAL_RESULT_H
