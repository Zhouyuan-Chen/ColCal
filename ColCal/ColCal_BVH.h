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
	ColCal_BVH(const std::vector<ColCal_Tri*>& tris_, int maxInterNum_ = 1, SplitMethod splitMethod_ = SplitMethod::EqualCounts)
		: objs_list(tris_), maxInterNum(maxInterNum_), splitMethod(splitMethod_){}
	void Build();
	void Build_Recursive(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node, SplitMethod splitMethod);
	
	void Build_Recursive_Middle(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node);
	void Build_Recursive_SAH(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node);
	void Build_Recursive_EqualCounts(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node);
	void Build_Recursive_HLBVH(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node);

	int maxInterNum;
	SplitMethod splitMethod;
	std::vector<ColCal_Tri*> objs_list;
	std::vector<ColCal_BVH_Node> nodes_list;
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

	bool isLeaf() { return leaf; }
	unsigned int getIndex() { return idx; }
	unsigned int getObjsNum() { return objs_num; }

	ColCal_Box box;
	bool leaf;
	unsigned int idx;
	unsigned int objs_num;
};

#endif // !BVH_H
