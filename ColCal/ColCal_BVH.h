#ifndef BVH_H
#define BVH_H

#include "ColCal.h"
#include "ColCal_Tri.h"
#include "ColCal_AABB.h"

enum class SplitMethod
{
	EqualCounts, Middle, SAH /* , HLBVH */
};

enum class BVH_AxisMethod {
	Variance, Difference
};

class ColCal_BVH_Node;

class ColCal_BVH {
public:
	ColCal_BVH(const std::vector<ColCal_Tri*>& tris_, unsigned int maxInterNum_ = 1, SplitMethod splitMethod_ = SplitMethod::EqualCounts, BVH_AxisMethod axisMethod_ = BVH_AxisMethod::Variance)
		: objs_list(tris_), maxInterNum(maxInterNum_), splitMethod(splitMethod_), axisMethod(axisMethod_){}
	void Build();
	void Build_Recursive(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node, SplitMethod splitMethod);
	void Build_Recursive_Middle(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node);
	void Build_Recursive_SAH(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node);
	void Build_Recursive_EqualCounts(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node);
	void Build_Recursive_HLBVH(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node);
	ColCal_BVH_Node* getLeftChild(ColCal_BVH_Node& node);
	ColCal_BVH_Node* getRightChild(ColCal_BVH_Node& node);
	int getOptimalAxis(const ColCal_BVH_Node& node, const unsigned int& left, const unsigned& right);
	int getMaxVarAxis(const ColCal_BVH_Node& node, const unsigned int& left, const unsigned& right);
	int getMaxDifAxis(const ColCal_BVH_Node& node, const unsigned int& left, const unsigned& right);
	inline void BVH_Sort(const unsigned int& left_index, const unsigned int& right_index, const int dim = 0) {
		if (dim == 0)
			std::sort(objs_list.begin() + left_index, objs_list.begin() + right_index - 1, &TriCompare_X);
		else if (dim == 1)
			std::sort(objs_list.begin() + left_index, objs_list.begin() + right_index - 1, &TriCompare_Y);
		else
			std::sort(objs_list.begin() + left_index, objs_list.begin() + right_index - 1, &TriCompare_Z);
	}

private:
	unsigned int maxInterNum;
	SplitMethod splitMethod;
	BVH_AxisMethod axisMethod;
	std::vector<ColCal_Tri*> objs_list;
	std::vector<ColCal_BVH_Node> nodes_list;
};

class ColCal_BVH_Node {
public:
	ColCal_BVH_Node() {
		box = ColCal_AABB();
		objs_num = 0;
		idx = 0;
		childs[0] = childs[1] = 0;
		leaf = false;
	}
	ColCal_BVH_Node(const ColCal_AABB& b, unsigned int Objs_num, unsigned Begin_index, bool Leaf = false) {
		this->box = ColCal_AABB(b);
		this->objs_num = objs_num;
		this->idx = Begin_index;
		childs[0] = childs[1] = -1;
		this->leaf = Leaf;
	}

	void setBox(ColCal_AABB&);
	ColCal_AABB getBox() const;
	void makeLeaf(unsigned int left, unsigned right);

	bool isLeaf() { return leaf; }
	unsigned int getIndex() { return idx; }
	unsigned int getObjsNum() { return objs_num; }
	unsigned int* getChildsIndex();

private:
	ColCal_AABB box;
	bool leaf;
	// relate to objs_list
	unsigned int idx;
	unsigned int objs_num;
	unsigned int childs[2];
};

#endif // !BVH_H
