#include "ColCal_KdTree.h"

ColCal_KdTree_Node::ColCal_KdTree_Node() {
	this->index = 0;
	this->axis = 0;
	this->objs_num = 0;
	this->split_value = ColCal_Max_Value;
	this->leaf = false;
}

void ColCal_KdTree_Node::MakeLeaf(unsigned int left_index, unsigned int right_index) {
	this->leaf = true;
	this->index = left_index;
	this->objs_num = right_index - left_index;
}

unsigned int* ColCal_KdTree_Node::getChildsIndex() {
	return this->childs;
}

int ColCal_KdTree_Node::getAxis() {
	return this->axis;
}

ColCal_DataType ColCal_KdTree_Node::getSplitValue() {
	return this->split_value;
}
// get split_index
unsigned int ColCal_KdTree_Node::getIndex() {
	return this->index;
}

void ColCal_KdTree::Build() {
	// initiate nodes_list
	if (nodes_list.size()) {
		nodes_list.clear();
	}

	if (!pts_list.size()) {
		return;
	}

	// find initial partition point
	ColCal_DataType Var_X, Var_Y, Var_Z, Mean_X, Mean_Y, Mean_Z;
	Var_X = Var_Y = Var_Z = Mean_X = Mean_Y = Mean_Z = 0.0;
	for (const auto& o : pts_list) {
		Var_X += o->x * o->x;
		Var_Y += o->y * o->y;
		Var_Z += o->z * o->z;
		Mean_X += o->x;
		Mean_Y += o->y;
		Mean_Z += o->z;
	}

	Var_X /= pts_list.size() * 1.0; // E(X^2)
	Mean_X /= pts_list.size() * 1.0; // E(X)
	Mean_X *= Mean_X; // E(X)^2
	Var_X -= Mean_X;

	Var_Y /= pts_list.size() * 1.0;
	Mean_Y /= pts_list.size() * 1.0;
	Mean_Y *= Mean_Y;
	Var_Y -= Mean_Y;

	Var_Z /= pts_list.size() * 1.0;
	Mean_Z /= pts_list.size() * 1.0;
	Mean_Z *= Mean_Z;
	Var_Z -= Mean_Z;

	int start_dim = Var_X > Var_Y ? 0 : (Var_Y > Var_Z ? 1 : 2);
	KdTree_Sort(0, pts_list.size(), start_dim);
	unsigned int split_index = pts_list.size() / 2;
	ColCal_DataType split_value = (*pts_list[split_index])[start_dim];

	ColCal_KdTree_Node root_node = ColCal_KdTree_Node(split_index, start_dim, split_value);
	nodes_list.push_back(root_node);

	// RecursiveBuild for right node
	RecursiveBuild(root_node, 0, pts_list.size(), 0);
}

void ColCal_KdTree::RecursiveBuild(ColCal_KdTree_Node& node, const unsigned int& left_index, const unsigned int& right_index, const int& depth) {
	if (right_index - left_index <= innerMaxNum || depth > MaxDepth) {
		node.MakeLeaf(left_index, right_index);
	}

	int cur_axis = (node.getAxis() + 1) % 3;

	// for left nodes
	unsigned int left_node_leftindex = left_index;
	unsigned int left_node_rightindex = node.getIndex() - 1;
	if (left_node_rightindex - left_node_leftindex > 0) {
		if (left_node_rightindex - left_node_leftindex <= innerMaxNum) {
			ColCal_KdTree_Node left_node = ColCal_KdTree_Node(left_node_leftindex, cur_axis);
			left_node.MakeLeaf(left_node_leftindex, left_node_rightindex);
			nodes_list.push_back(left_node);
			node.getChildsIndex()[0] = nodes_list.size() - 1;
		}
		else {
			KdTree_Sort(left_node_leftindex, left_node_rightindex, cur_axis);
			ColCal_DataType split_index = (left_node_leftindex + left_node_rightindex) / 2;
			ColCal_DataType split_value = (*pts_list[split_index])[cur_axis];
			ColCal_KdTree_Node left_node = ColCal_KdTree_Node(split_index, cur_axis, split_value);
			RecursiveBuild(left_node, left_node_leftindex, left_node_rightindex, depth + 1);
			nodes_list.push_back(left_node);
			node.getChildsIndex()[0] = nodes_list.size() - 1;
		}
	}

	// for right nodes
	unsigned int right_node_leftindex = node.getIndex() + 1;
	unsigned int right_node_rightindex = right_index;
	if (right_node_rightindex - right_node_leftindex > 0) {
		if (right_node_rightindex - right_node_leftindex <= innerMaxNum) {
			ColCal_KdTree_Node right_node = ColCal_KdTree_Node(right_node_leftindex, cur_axis);
			right_node.MakeLeaf(right_node_leftindex, right_node_rightindex);
			nodes_list.push_back(right_node);
			node.getChildsIndex()[1] = nodes_list.size() - 1;
		}
		else {
			KdTree_Sort(right_node_leftindex, right_node_rightindex, cur_axis);
			ColCal_DataType split_index = (right_node_leftindex + right_node_rightindex) / 2;
			ColCal_DataType split_value = (*pts_list[split_index])[cur_axis];
			ColCal_KdTree_Node right_node = ColCal_KdTree_Node(split_index, cur_axis, split_value);
			RecursiveBuild(right_node, right_node_leftindex, right_node_rightindex, depth + 1);
			nodes_list.push_back(right_node);
			node.getChildsIndex()[1] = nodes_list.size() - 1;
		}
	}

}

unsigned int ColCal_KdTree::NearestSearch() {
	return 0;
}