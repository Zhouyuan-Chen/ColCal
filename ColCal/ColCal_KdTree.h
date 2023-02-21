#ifndef COLCAL_KDTREE
#define COLCAL_KDTREE

#include "ColCal.h"
#include "ColCal_Tri.h"
#include "ColCal_Box.h"

class ColCal_KdTree_Node {
public:
	ColCal_KdTree_Node();
	ColCal_KdTree_Node(unsigned int index_, unsigned int cur_axis_, ColCal_DataType split_value_ = ColCal_Max_Value, unsigned int objs_num_ = 0, bool leaf_ = false)
		: index(index_), axis(cur_axis_), split_value(split_value_), objs_num(objs_num_), leaf(leaf_) {
		this->childs[0] = this->childs[1] = -1;
	}
	int getAxis();
	ColCal_DataType getSplitValue();
	unsigned int getIndex(); 
	unsigned int* getChildsIndex();
	void MakeLeaf(unsigned int left_index, unsigned int right_index);
	
private:
	unsigned int index;
	unsigned int objs_num;
	unsigned int axis;
	ColCal_DataType split_value;
	unsigned int childs[2];
	bool leaf;
};

class ColCal_KdTree {
public:
	ColCal_KdTree(const std::vector<ColCal_Point*> pts, unsigned int innerMaxNum_ = 128, unsigned int MaxDepth_ = 20)
		:pts_list(pts), innerMaxNum(innerMaxNum_), MaxDepth(MaxDepth_) {}
	void Build();
	void RecursiveBuild(ColCal_KdTree_Node& node,const unsigned int& left_index, const unsigned int& right_index, const int& depth);
	inline void KdTree_Sort(const unsigned int& left_index, const unsigned int& right_index, const int& dim) {
		if (dim == 0)
			std::sort(pts_list.begin() + left_index, pts_list.begin() + right_index - 1, &PointCompare_X);
		else if (dim == 1)
			std::sort(pts_list.begin() + left_index, pts_list.begin() + right_index - 1, &PointCompare_Y);
		else
			std::sort(pts_list.begin() + left_index, pts_list.begin() + right_index - 1, &PointCompare_Z);
	}
	unsigned int NearestSearch();
private:
	std::vector<ColCal_KdTree_Node> nodes_list;
	std::vector<ColCal_Point*> pts_list;
	unsigned int innerMaxNum;
	unsigned int MaxDepth;
};

#endif