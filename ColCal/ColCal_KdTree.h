#ifndef COLCAL_KDTREE
#define COLCAL_KDTREE

#include "ColCal.h"
#include "ColCal_Tri.h"
#include "ColCal_Box.h"

class KdTree_Node {
public:
	KdTree_Node();
	KdTree_Node(unsigned int index, unsigned int axis, ColCal_DataType split_value);

private:
	unsigned int index;
	unsigned int childs[2];
	unsigned int axis;
	ColCal_DataType split_value;
};

class ColCal_KdTree {
public:
	ColCal_KdTree(const std::vector<ColCal_Point*> pts) :pts_list(pts){}
	void Build();
	// void search nearest points();
	void Insert(ColCal_Point* p);
private:
	std::vector<KdTree_Node> nodes_list;
	std::vector<ColCal_Point*> pts_list;
};

#endif