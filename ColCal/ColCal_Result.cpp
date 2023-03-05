#include "ColCal_Result.h"

ColCal_Result_BVH::ColCal_Result_BVH() {
	result.clear();
	distance = 0.0;
	BVH_A = nullptr;
	BVH_B = nullptr;
}

bool ColCal_Result_BVH::CalculateCollide(ColCal_BVH& A, ColCal_Mat4& R1, ColCal_Vec4& T1, ColCal_BVH& B, ColCal_Mat4& R2, ColCal_Vec4& T2) {
	// initiate M for Collision
	//    T1*R1*A T2*R2*B
	// -> R1*A (T1^-1)*T2*R2*B
	// -> A (R1^-1)*(T1^-1)*T2*R2*B
	// -> A M*B [M=(R1^-1)*(T1^-1)*T2*R2]

	ColCal_Mat4 M, T_;
	T_[0][3] = T2[0]; T_[1][3] = T2[1]; T_[2][3] = T2[2]; // T2
	M = T_ * M; // T2*R2
	T_[0][3] = T1[0]; T_[1][3] = T1[1]; T_[2][3] = T1[2]; // T1
	M = T_.getInverse() * M; // (T1^-1)*T2*R2
	M = R1.getInverse() * M; // (R1^-1)*(T1^-1)*T2*R2

	// setup
	BVH_A = &A;
	BVH_B = &B;
	if (!A.Empty() && !B.Empty()){
		result.clear();
		CalculateCollide(A.getNode(0), B.getNode(0), M);
	}
	return result.size() != 0;
}

void ColCal_Result_BVH::CalculateCollide(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M) {
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
			CalculateCollide(A, *B_left_node, M);
		if (B_right_node)
			CalculateCollide(A, *B_right_node, M);
	}
	else if (!A.isLeaf() && B.isLeaf()) {
		ColCal_BVH_Node_AABB* A_left_node = BVH_A->getLeftChild(A);
		ColCal_BVH_Node_AABB* A_right_node = BVH_A->getRightChild(A);
		if (A_left_node)
			CalculateCollide(*A_left_node, B, M);
		if (A_right_node)
			CalculateCollide(*A_right_node, B, M);
	}
	else if (!A.isLeaf() && !B.isLeaf()) {
		ColCal_BVH_Node_AABB* A_left_node = BVH_A->getLeftChild(A);
		ColCal_BVH_Node_AABB* A_right_node = BVH_A->getRightChild(A);
		ColCal_BVH_Node_AABB* B_left_node = BVH_B->getLeftChild(B);
		ColCal_BVH_Node_AABB* B_right_node = BVH_B->getRightChild(B);
		// Al-Bl Al-Br Ar-Bl Ar-Br
		if (A_left_node) {
			if(B_left_node)
				CalculateCollide(*A_left_node, *B_left_node, M);
			if (B_right_node)
				CalculateCollide(*A_left_node, *B_right_node, M);
		}
		if (A_right_node) {
			if (B_left_node)
				CalculateCollide(*A_right_node, *B_left_node, M);
			if (B_right_node)
				CalculateCollide(*A_right_node, *B_right_node, M);
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

void ColCal_Result_BVH::CalculatePairsDistance(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M) {
	unsigned int A_begin_index = A.getIndex();
	unsigned int A_end_index = A_begin_index + A.getObjsNum();
	unsigned int B_begin_index = B.getIndex();
	unsigned int B_end_index = B_begin_index + B.getObjsNum();

	for (unsigned int A_index = A_begin_index; A_index < A_end_index; A_index++) {
		for (unsigned int B_index = B_begin_index; B_index < B_end_index; B_index++) {
			ColCal_Tri* tri1 = BVH_A->getObj(A_index);
			ColCal_Tri* tri2 = BVH_B->getObj(B_index);
			// calculate tri distance
			ColCal_DataType dis = tri1->tri_dis(*tri2, M);
			this->distance = ColCal_Min(this->distance, dis);
		}
	}
}

ColCal_DataType ColCal_Result_BVH::CalculateDistance(ColCal_BVH& A, ColCal_Mat4& R1, ColCal_Vec4& T1, ColCal_BVH& B, ColCal_Mat4& R2, ColCal_Vec4& T2) {
	// initiate M for Collision
	//    T1*R1*A T2*R2*B
	// -> R1*A (T1^-1)*T2*R2*B
	// -> A (R1^-1)*(T1^-1)*T2*R2*B
	// -> A M*B [M=(R1^-1)*(T1^-1)*T2*R2]

	ColCal_Mat4 M, T_;
	T_[0][3] = T2[0]; T_[1][3] = T2[1]; T_[2][3] = T2[2]; // T2
	M = T_ * M; // T2*R2
	T_[0][3] = T1[0]; T_[1][3] = T1[1]; T_[2][3] = T1[2]; // T1
	M = T_.getInverse() * M; // (T1^-1)*T2*R2
	M = R1.getInverse() * M; // (R1^-1)*(T1^-1)*T2*R2

	// setup
	BVH_A = &A;
	BVH_B = &B;
	if (!A.Empty() && !B.Empty()) {
		this->distance = ColCal_Max_Value;
		CalculateDistance(A.getNode(0), B.getNode(0), M);
	}
	return this->distance;
}

void ColCal_Result_BVH::CalculateDistance(ColCal_BVH_Node_AABB& A, ColCal_BVH_Node_AABB& B, ColCal_Mat4& M) {
	if (A.getBox().getProximateDis(B.getBox(), M) > this->distance)
		return;

	if (A.isLeaf() && B.isLeaf()) {
		// Calculate Pair's tris' distance
		CalculatePairsDistance(A, B, M);
	}
	else if (A.isLeaf() && !B.isLeaf()) {
		ColCal_BVH_Node_AABB* B_left_node = BVH_B->getLeftChild(B);
		ColCal_BVH_Node_AABB* B_right_node = BVH_B->getRightChild(B);
		if (B_left_node)
			CalculateDistance(A, *B_left_node, M);
		if (B_right_node)
			CalculateDistance(A, *B_right_node, M);
	}
	else if (!A.isLeaf() && B.isLeaf()) {
		ColCal_BVH_Node_AABB* A_left_node = BVH_A->getLeftChild(A);
		ColCal_BVH_Node_AABB* A_right_node = BVH_A->getRightChild(A);
		if (A_left_node)
			CalculateDistance(*A_left_node, B, M);
		if (A_right_node)
			CalculateDistance(*A_right_node, B, M);
	}
	else if (!A.isLeaf() && !B.isLeaf()) {
		ColCal_BVH_Node_AABB* A_left_node = BVH_A->getLeftChild(A);
		ColCal_BVH_Node_AABB* A_right_node = BVH_A->getRightChild(A);
		ColCal_BVH_Node_AABB* B_left_node = BVH_B->getLeftChild(B);
		ColCal_BVH_Node_AABB* B_right_node = BVH_B->getRightChild(B);
		// Al-Bl Al-Br Ar-Bl Ar-Br
		if (A_left_node) {
			if (B_left_node)
				CalculateDistance(*A_left_node, *B_left_node, M);
			if (B_right_node)
				CalculateDistance(*A_left_node, *B_right_node, M);
		}
		if (A_right_node) {
			if (B_left_node)
				CalculateDistance(*A_right_node, *B_left_node, M);
			if (B_right_node)
				CalculateDistance(*A_right_node, *B_right_node, M);
		}
	}
}

unsigned int ColCal_Result_BVH::CollisionNum() {
	return result.size();
}

ColCal_DataType ColCal_Result_BVH::getDistance() {
	return this->distance;
}