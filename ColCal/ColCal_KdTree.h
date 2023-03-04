#ifndef COLCAL_KDTREE_H
#define COLCAL_KDTREE_H

#include "ColCal.h"
#include "ColCal_Tri.h"
#include "ColCal_AABB.h"

enum class SearchMethod {
	NormalSearch, BBF
};

class ColCal_KdTree_Node {
public:
	ColCal_KdTree_Node();
	ColCal_KdTree_Node(unsigned int index_, unsigned int cur_axis_, ColCal_DataType split_value_ = ColCal_Max_Value, unsigned int objs_num_ = 0, bool leaf_ = false, ColCal_DataType BBF_Value_ = ColCal_Max_Value)
		: index(index_), axis(cur_axis_), split_value(split_value_), objs_num(objs_num_), leaf(leaf_), BBF_Value(BBF_Value_) {
		this->childs[0] = this->childs[1] = -1;
	}
	int getAxis();
	ColCal_DataType getSplitValue();
	ColCal_DataType getBBF_Value()const;
	void setBBF_Value(const ColCal_DataType& BBF_Value_);
	unsigned int getIndex()const;
	unsigned int* getChildsIndex();
	unsigned int getObjsNum()const;
	void MakeLeaf(unsigned int left_index, unsigned int right_index);
	bool isLeaf() const;
	
private:
	unsigned int index;
	unsigned int objs_num;
	unsigned int axis;
	ColCal_DataType split_value;
	unsigned int childs[2];
	bool leaf;
	ColCal_DataType BBF_Value;
};

inline bool KdNodeCompareBBF(ColCal_KdTree_Node* A, ColCal_KdTree_Node* B) {
	return A->getBBF_Value() > B->getBBF_Value();
}

class ColCal_KdTree {
public:
	ColCal_KdTree(const std::vector<ColCal_Point*> pts, unsigned int innerMaxNum_ = 128, unsigned int MaxDepth_ = 20)
		:pts_list(pts), innerMaxNum(innerMaxNum_), MaxDepth(MaxDepth_) {}
	void Build();
	void RecursiveBuild(ColCal_KdTree_Node& node,const unsigned int& left_index, const unsigned int& right_index, const int& depth);
	unsigned int NearestSearch(const ColCal_Point*& target_point, SearchMethod method = SearchMethod::BBF);
	unsigned int NormalSearch(const ColCal_Point*& target_point);
	unsigned int BBFSearch(const ColCal_Point*& target_point);
	bool Existed(const std::vector<ColCal_KdTree_Node*>& nodes, const unsigned int& node_index);
	inline void KdTree_Sort(const unsigned int& left_index, const unsigned int& right_index, const int& dim) {
		if (dim == 0)
			std::sort(pts_list.begin() + left_index, pts_list.begin() + right_index - 1, &PointCompare_X);
		else if (dim == 1)
			std::sort(pts_list.begin() + left_index, pts_list.begin() + right_index - 1, &PointCompare_Y);
		else
			std::sort(pts_list.begin() + left_index, pts_list.begin() + right_index - 1, &PointCompare_Z);
	}
	inline void KdTree_Sort_BBF(std::vector<ColCal_KdTree_Node*>& list) {
		std::sort(list.begin(), list.end(), &KdNodeCompareBBF);
	}
	ColCal_DataType Calculate_BBF_Value(const ColCal_KdTree_Node& A, const ColCal_Point& p) {
		// manhatte
		if (!A.isLeaf()){
			ColCal_Point A_p = *pts_list[A.getIndex()];
			return ColCal_Fabs(A_p.x - p.x) + ColCal_Fabs(A_p.y - p.y) + ColCal_Fabs(A_p.z - p.z);
		}
		else {
			ColCal_Point mid_p = *pts_list[(A.getIndex() * 2 + A.getObjsNum()) / 2];
			return ColCal_Fabs(mid_p.x - p.x) + ColCal_Fabs(mid_p.y - p.y) + ColCal_Fabs(mid_p.z - p.z);
		}
	}

private:
	std::vector<ColCal_KdTree_Node> nodes_list;
	std::vector<ColCal_Point*> pts_list;
	unsigned int innerMaxNum;
	unsigned int MaxDepth;
	SearchMethod searchMethod;
};

#endif