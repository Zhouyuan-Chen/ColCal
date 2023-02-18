#ifndef BVH_H
#define BVH_H

#include "ColCal.h"
#include "ColCal_Tri.h"
#include "ColCal_Box.h"

enum class SplitMethod
{
	EqualCounts, Middle, SAH, HLBVH
};

class ColCal_BVH {
public:
	ColCal_BVH(const std::vector<ColCal_Tri*>& tris_, int maxInterNum_, SplitMethod splitMethod_ = SplitMethod::EqualCounts)
		: tris(tris_), maxInterNum(maxInterNum_), splitMethod(splitMethod_){}
	void Build();
	void Build_Recursive(int left_index, int right_index, ColCal_BVH_Node* node, SplitMethod splitMethod) {
		switch (splitMethod)
		{
		case SplitMethod::EqualCounts:
			Build_Recursive_EqualCounts(left_index, right_index, node);
			break;
		case SplitMethod::Middle:
			Build_Recursive_Middle(left_index, right_index, node);
			break;
		case SplitMethod::SAH:
			Build_Recursive_SAH(left_index, right_index, node);
			break;
		case SplitMethod::HLBVH:
			Build_Recursive_HLBVH(left_index, right_index, node);
			break;
		default:
			Build_Recursive_EqualCounts(left_index, right_index, node);
			break;
		}
	}
	
	void Build_Recursive_Middle(int left_index, int right_index, ColCal_BVH_Node* node);
	void Build_Recursive_SAH(int left_index, int right_index, ColCal_BVH_Node* node);
	void Build_Recursive_EqualCounts(int left_index, int right_index, ColCal_BVH_Node* node);
	void Build_Recursive_HLBVH(int left_index, int right_index, ColCal_BVH_Node* node);

	int maxInterNum;
	SplitMethod splitMethod;
	std::vector<ColCal_Tri*> tris;
};

class ColCal_BVH_Node {
public:
	ColCal_BVH_Node(){
		box = ColCal_Box();
		objs_num = 0;
		idx = 0;
		leaf = false;
	}
	ColCal_BVH_Node(const ColCal_Box& b, unsigned int Objs_num, unsigned Idx, bool Leaf = false) {
		this->box = ColCal_Box(b);
		this->objs_num = objs_num;
		this->idx = Idx;
		this->leaf = Leaf;
	}

	void setBox(ColCal_Box&);
	void makeLeaf(unsigned int left, unsigned right);
	void makeNode();

	bool isLeaf() { return leaf; }
	unsigned int getIndex() { return idx; }
	unsigned int getObjsNum() { return objs_num; }

	ColCal_Box box;
	bool leaf;
	unsigned int idx;
	unsigned int objs_num;
};

#endif // !BVH_H
