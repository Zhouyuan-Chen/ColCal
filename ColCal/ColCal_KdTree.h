#ifndef COLCAL_KDTREE
#define COLCAL_KDTREE

#include "ColCal.h"
#include "ColCal_Tri.h"
#include "ColCal_Box.h"

class KdTree_Node {
public:
	KdTree_Node();
	KdTree_Node(unsigned int index_, unsigned int axis_, unsigned int objs_num_, ColCal_DataType split_value_ = ColCal_Max_Value, bool leaf_ = false)
		: index(index_), axis(axis_), objs_num(objs_num_), split_value(split_value_), leaf(leaf_){}
	void MakeLeaf(unsigned int Index, unsigned int objs_num);
private:
	unsigned int index;
	unsigned int objs_num;
	unsigned int axis;
	ColCal_DataType split_value;
	bool leaf;
};

class ColCal_KdTree {
public:
	ColCal_KdTree(const std::vector<ColCal_Point*> pts) :pts_list(pts){}
	void Build();
	void RecursiveBuild();
	// void search nearest points();
private:
	std::vector<KdTree_Node> nodes_list;
	std::vector<ColCal_Point*> pts_list;
};

#endif