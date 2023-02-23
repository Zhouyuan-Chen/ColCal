#include "ColCal_KdTree.h"

ColCal_KdTree_Node::ColCal_KdTree_Node() {
	this->index = 0;
	this->axis = 0;
	this->objs_num = 0;
	this->split_value = ColCal_Max_Value;
	this->leaf = false;
	this->childs[0] = this->childs[1] = -1;
}

void ColCal_KdTree_Node::MakeLeaf(unsigned int left_index, unsigned int right_index) {
	this->leaf = true;
	this->index = left_index;
	this->objs_num = right_index - left_index;
	this->childs[0] = this->childs[1] = -1;
}

unsigned int* ColCal_KdTree_Node::getChildsIndex() {
	return this->childs;
}

// get split_index
unsigned int ColCal_KdTree_Node::getIndex()const {
	return this->index;
}

unsigned int ColCal_KdTree_Node::getObjsNum()const {
	return this->objs_num;
}

int ColCal_KdTree_Node::getAxis() {
	return this->axis;
}

ColCal_DataType ColCal_KdTree_Node::getSplitValue() {
	return this->split_value;
}

bool ColCal_KdTree_Node::isLeaf() const {
	return this->leaf;
}

ColCal_DataType ColCal_KdTree_Node::getBBF_Value()const {
	return BBF_Value;
}

void ColCal_KdTree_Node::setBBF_Value(const ColCal_DataType& BBF_Value_) {
	this->BBF_Value = BBF_Value_;
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

	// put all points which equal to split value to right nodes
	while (split_index > 0 && (*pts_list[split_index])[start_dim] == (*pts_list[split_index - 1])[start_dim]) {
		split_index--;
	}

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

			// put all points which equal to split value to right nodes
			while (split_index > 0 && (*pts_list[split_index])[cur_axis] == (*pts_list[split_index - 1])[cur_axis]) {
				split_index--;
			}

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

			// put all points which equal to split value to right nodes
			while (split_index > 0 && (*pts_list[split_index])[cur_axis] == (*pts_list[split_index - 1])[cur_axis]) {
				split_index--;
			}

			ColCal_KdTree_Node right_node = ColCal_KdTree_Node(split_index, cur_axis, split_value);
			RecursiveBuild(right_node, right_node_leftindex, right_node_rightindex, depth + 1);
			nodes_list.push_back(right_node);
			node.getChildsIndex()[1] = nodes_list.size() - 1;
		}
	}

}

unsigned int ColCal_KdTree::NearestSearch(const ColCal_Point*& target_point, SearchMethod method = SearchMethod::BBF) {
	switch (method)
	{
	case SearchMethod::NormalSearch:
		return NormalSearch(target_point);
	case SearchMethod::BBF:
		return BBFSearch(target_point);
	default:
		return NormalSearch(target_point);
	}
}

unsigned int ColCal_KdTree::NormalSearch(const ColCal_Point*& target_point) {
	if (nodes_list.empty())
		return -1;

	ColCal_Point target_p = *target_point;

	// initiate path vector and nearest_dis
	std::vector<ColCal_KdTree_Node*> path;
	path.push_back(&nodes_list[0]);
	ColCal_DataType nearest_distance = ColCal_Max_Value;
	unsigned int nearest_index = -1;

	// traverse kd-tree to find target_point's domain
	ColCal_KdTree_Node traverse = nodes_list[0];
	while (!traverse.isLeaf()) {
		ColCal_DataType spilt_value = traverse.getSplitValue();
		int dim = traverse.getAxis();
		
		if (target_p[dim] < spilt_value)
			path.push_back(&nodes_list[traverse.getChildsIndex()[0]]);	
		else 
			path.push_back(&nodes_list[traverse.getChildsIndex()[1]]);
		
		traverse = *path.back();
	}

	// backtrack to modify our nearest point
	ColCal_Point nearest_point = *pts_list[nearest_index];
	std::vector<ColCal_KdTree_Node*> traversed_nodes_list;
	while (!path.empty()) {
		ColCal_KdTree_Node*& last_path_node = path.back();
		path.pop_back();
		traversed_nodes_list.push_back(last_path_node);
		
		// if is a leaf, then traverse the leaf and find the nearest point
		if (last_path_node->isLeaf()) {
			unsigned int left_index, right_index;
			left_index = last_path_node->getIndex();
			right_index = left_index + last_path_node->getObjsNum();
			for (unsigned int i = left_index; i < right_index; i++) {
				ColCal_Point temp_p = *pts_list[i];
				ColCal_DataType distance = temp_p.getDistance(target_p);
				if (distance < nearest_distance) {
					nearest_index = i;
					nearest_point = *pts_list[nearest_index];
				}
			}
		}
		// if it is not a leaf, then add one child node and test if need to add another child node
		else {
			int cur_axis = last_path_node->getAxis();
			ColCal_DataType split_value = last_path_node->getSplitValue();
			
			if (target_p[cur_axis] < split_value && Existed(traversed_nodes_list, last_path_node->getChildsIndex()[0])) {
				// add left child node
				path.push_back(&nodes_list[last_path_node->getChildsIndex()[0]]);
			}
			else if (target_p[cur_axis] >= split_value && Existed(traversed_nodes_list, last_path_node->getChildsIndex()[1])) {
				// add right child node
				path.push_back(&nodes_list[last_path_node->getChildsIndex()[1]]);
			}
			
			// check if need to add another child node
			ColCal_DataType distanceToAnotherchild = ColCal_Fabs(split_value - target_p[cur_axis]);
			if (target_p[cur_axis] < split_value && distanceToAnotherchild < nearest_distance && Existed(traversed_nodes_list, last_path_node->getChildsIndex()[1])) {
				// add right child node
				path.push_back(&nodes_list[last_path_node->getChildsIndex()[1]]);
			}
			else if (target_p[cur_axis] >= split_value && distanceToAnotherchild < nearest_distance && Existed(traversed_nodes_list, last_path_node->getChildsIndex()[0])) {
				// add right child node
				path.push_back(&nodes_list[last_path_node->getChildsIndex()[0]]);
			}
		}
	}

	return nearest_index;
}

unsigned int ColCal_KdTree::BBFSearch(const ColCal_Point*& target_point) {
	if (nodes_list.empty())
		return -1;

	ColCal_Point target_p = *target_point;

	// initiate path vector and nearest_dis
	std::vector<ColCal_KdTree_Node*> path;
	path.push_back(&nodes_list[0]);
	ColCal_DataType nearest_distance = ColCal_Max_Value;
	unsigned int nearest_index = -1;

	// traverse kd-tree to find target_point's domain
	ColCal_KdTree_Node* traverse = &nodes_list[0];
	while (!traverse->isLeaf()) {
		ColCal_DataType spilt_value = traverse->getSplitValue();
		int dim = traverse->getAxis();
		ColCal_DataType BBF = Calculate_BBF_Value(*traverse, target_p);
		traverse->setBBF_Value(BBF);

		if (target_p[dim] < spilt_value)
			path.push_back(&nodes_list[traverse->getChildsIndex()[0]]);
		else
			path.push_back(&nodes_list[traverse->getChildsIndex()[1]]);

		traverse = path.back();
	}
	// this domain should be traversed firstly
	traverse->setBBF_Value(ColCal_Min_Value);

	// backtrack to modify our nearest point
	ColCal_Point nearest_point = *pts_list[nearest_index];
	std::vector<ColCal_KdTree_Node*> traversed_nodes_list;
	while (!path.empty()) {
		KdTree_Sort_BBF(path);
		ColCal_KdTree_Node*& last_path_node = path.back();
		path.pop_back();
		traversed_nodes_list.push_back(last_path_node);

		// if is a leaf, then traverse the leaf and find the nearest point
		if (last_path_node->isLeaf()) {
			unsigned int left_index, right_index;
			left_index = last_path_node->getIndex();
			right_index = left_index + last_path_node->getObjsNum();
			for (unsigned int i = left_index; i < right_index; i++) {
				ColCal_Point temp_p = *pts_list[i];
				ColCal_DataType distance = temp_p.getDistance(target_p);
				if (distance < nearest_distance) {
					nearest_index = i;
					nearest_point = *pts_list[nearest_index];
				}
			}
		}
		// if it is not a leaf, then add one child node and test if need to add another child node
		else {
			int cur_axis = last_path_node->getAxis();
			ColCal_DataType split_value = last_path_node->getSplitValue();

			if (target_p[cur_axis] < split_value && !Existed(traversed_nodes_list, last_path_node->getChildsIndex()[0])) {
				// add left child node
				ColCal_DataType BBF = Calculate_BBF_Value(nodes_list[last_path_node->getChildsIndex()[0]],target_p);
				nodes_list[last_path_node->getChildsIndex()[0]].setBBF_Value(BBF);
				path.push_back(&nodes_list[last_path_node->getChildsIndex()[0]]);
			}
			else if (target_p[cur_axis] >= split_value && !Existed(traversed_nodes_list, last_path_node->getChildsIndex()[1])) {
				// add right child node
				ColCal_DataType BBF = Calculate_BBF_Value(nodes_list[last_path_node->getChildsIndex()[1]], target_p);
				nodes_list[last_path_node->getChildsIndex()[1]].setBBF_Value(BBF);
				path.push_back(&nodes_list[last_path_node->getChildsIndex()[1]]);
			}

			// check if need to add another child node
			ColCal_DataType distanceToAnotherchild = ColCal_Fabs(split_value - target_p[cur_axis]);
			if (target_p[cur_axis] < split_value && distanceToAnotherchild < nearest_distance && !Existed(traversed_nodes_list, last_path_node->getChildsIndex()[1])) {
				// add right child node
				ColCal_DataType BBF = Calculate_BBF_Value(nodes_list[last_path_node->getChildsIndex()[1]], target_p);
				nodes_list[last_path_node->getChildsIndex()[1]].setBBF_Value(BBF);
				path.push_back(&nodes_list[last_path_node->getChildsIndex()[1]]);
			}
			else if (target_p[cur_axis] >= split_value && distanceToAnotherchild < nearest_distance && !Existed(traversed_nodes_list, last_path_node->getChildsIndex()[0])) {
				// add right child node
				ColCal_DataType BBF = Calculate_BBF_Value(nodes_list[last_path_node->getChildsIndex()[0]], target_p);
				nodes_list[last_path_node->getChildsIndex()[0]].setBBF_Value(BBF);
				path.push_back(&nodes_list[last_path_node->getChildsIndex()[0]]);
			}
		}
	}

	return nearest_index;
}

bool ColCal_KdTree::Existed(const std::vector<ColCal_KdTree_Node*>& nodes, const unsigned int& node_index) {
	if (node_index == -1)
		return false;

	for (const ColCal_KdTree_Node* n : nodes) {
		if (n->getIndex() == node_index)
			return true;
	}
	return false;
}