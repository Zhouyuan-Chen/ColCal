#include "ColCal_Result.h"

// files need to be updated
// * ColCal_AABB collide
// * ColCal_Result Collide
// * ColCal_Result Distance
// * ColCal_Result Clock

bool ColCal_Result_BVH::Collide(ColCal_BVH& A, ColCal_Mat4& R1, ColCal_Vec4& T1, ColCal_BVH& B, ColCal_Mat4& R2, ColCal_Vec4& T2) {
	// initiate M for Collision


	// setup
	BVH_A = &A;
	BVH_B = &B;
	ColCal_Mat4 M;
	if (!A.Empty() && !B.Empty()){
		result.clear();
		Collide(A.getNode(0), B.getNode(0), M);
	}
	return result.size() != 0;
}

void ColCal_Result_BVH::Collide(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M) {
	if (!(A.getBox().collide(B.getBox(), M)))
		return;

	if (A.isLeaf() && B.isLeaf()) {
		if (A.getBox().collide(B.getBox(), M)) {
			// calculate
			CalculatePairs(A, B, M);
		}
	}
	else if (A.isLeaf() && !B.isLeaf()) {
		ColCal_BVH_Node_AABB* B_left_node = BVH_B->getLeftChild(B);
		ColCal_BVH_Node_AABB* B_right_node = BVH_B->getRightChild(B);
		if (B_left_node)
			Collide(A, *B_left_node, M);
		if (B_right_node)
			Collide(A, *B_right_node, M);
	}
	else if (!A.isLeaf() && B.isLeaf()) {
		ColCal_BVH_Node_AABB* A_left_node = BVH_A->getLeftChild(A);
		ColCal_BVH_Node_AABB* A_right_node = BVH_A->getRightChild(A);
		if (A_left_node)
			Collide(*A_left_node, B, M);
		if (A_right_node)
			Collide(*A_right_node, B, M);
	}
	else if (!A.isLeaf() && !B.isLeaf()) {
		ColCal_BVH_Node_AABB* A_left_node = BVH_A->getLeftChild(A);
		ColCal_BVH_Node_AABB* A_right_node = BVH_A->getRightChild(A);
		ColCal_BVH_Node_AABB* B_left_node = BVH_B->getLeftChild(B);
		ColCal_BVH_Node_AABB* B_right_node = BVH_B->getRightChild(B);
		// Al-Bl Al-Br Ar-Bl Ar-Br
		if (A_left_node) {
			if(B_left_node)
				Collide(*A_left_node, *B_left_node, M);
			if (B_right_node)
				Collide(*A_left_node, *B_right_node, M);
		}
		if (A_right_node) {
			if (B_left_node)
				Collide(*A_right_node, *B_left_node, M);
			if (B_right_node)
				Collide(*A_right_node, *B_right_node, M);
		}
	}
}

void ColCal_Result_BVH::CalculatePairs(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M) {
	unsigned int A_begin_index = A.getIndex();
	unsigned int A_end_index = A_begin_index + A.getObjsNum();
	unsigned int B_begin_index = B.getIndex();
	unsigned int B_end_index = B_begin_index + B.getObjsNum();

	for (unsigned int A_index = A_begin_index; A_index < A_end_index; A_index++) {
		for (unsigned int B_index = B_begin_index; B_index < B_end_index; B_index++) {
			ColCal_Tri* tri1 = BVH_A->getObj(A_index);
			ColCal_Tri* tri2 = BVH_B->getObj(B_index);
			if (tri1->collide(*tri2, M))
				result.push_back(std::pair<unsigned int, unsigned int>(tri1->idx, tri2->idx));
		}
	}
}

ColCal_DataType ColCal_Result_BVH::Distance(ColCal_BVH& A, ColCal_Mat4& R1, ColCal_Vec4& T1, ColCal_BVH& B, ColCal_Mat4& R2, ColCal_Vec4& T2) {

}

void ColCal_Result_BVH::Distance(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M) {

}

unsigned int ColCal_Result_BVH::CollisionNum() {
	return result.size();
}