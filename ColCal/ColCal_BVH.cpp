#include "ColCal_BVH.h"
/*
void build(const std::vector<Intersectable *> &objects)
• Create new vector for Intersectable pointer copies
• Create new vector for the nodes
• Create Root node
• worldBox = AABB(); // world bounding box
• For each intersectable[i] in objects
– worldBox.include(intersectable[i] bounding box)
– Objs.push_back(intersectable[i])
• EndFor
• Set world bounding box to root node
• build_recursive(0, Objs.size(), root, 0);
The declaration was: void build_recursive(int left_index, int right_index, BVHNode *node, int depth)
*/

void ColCal_BVH::Build() {
	// clear vector
	if (nodes_list.size())
		nodes_list.clear();

	// create the root node and World_Box for the root node
	ColCal_BVH_Node root_node = ColCal_BVH_Node();
	ColCal_Box World_Box = ColCal_Box();

	if (objs_list.size() != 0) {
		World_Box = ColCal_Box(*objs_list[0]);
	}
	for (const auto& o : objs_list) {
		World_Box.Include(*o);
	}

	// bond the World_Box to the root node
	root_node.setBox(World_Box);
	nodes_list.push_back(root_node);
	Build_Recursive(0, objs_list.size(), root_node, this->splitMethod);
}

void ColCal_BVH::Build_Recursive(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node, SplitMethod splitMethod) {
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

/*
void build_recursive(int left_index, int right_index, BVHNode *node, int depth)
• If ((right_index – left_index) <= Threshold || (other termination criteria))
– Initiate current node as a leaf with primitives from Objs[left_index] to Objs[right_index]
• Else
– Split intersectables into left and right by finding a split_index
• Make sure that neither left nor right is completely empty
– Calculate bounding boxes of left and right sides
– Create two new nodes, leftNode and rightNode and assign bounding boxes
– Initiate current node as an interior node with leftNode and rightNode as children
– build_recursive(left_index, split_index, leftNode, depth + 1)
– build_recursive(split_index, right_index, rightNode, depth + 1)
• EndIf
*/

void ColCal_BVH::Build_Recursive_EqualCounts(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node) {
	if ((right_index - left_index) <= this->maxInterNum) {
		// make leaf
		node.makeLeaf(left_index, right_index);
	}

	// find a largest dimension
	ColCal_DataType Range_X = node.box.Max[0] - node.box.Min[0];
	ColCal_DataType Range_Y = node.box.Max[1] - node.box.Min[1];
	ColCal_DataType Range_Z = node.box.Max[2] - node.box.Min[2];

	int dim = 0;
	ColCal_DataType Range_Max = Range_X;
	if (Range_Y > Range_Max) {
		Range_Max = Range_Y;
		dim = 1;
	}
	if (Range_Z > Range_Max) {
		dim = 2;
	}

	if (dim == 0)
		std::sort(objs_list.begin() + left_index, objs_list.begin() + right_index - 1, &TriCompare_X);
	else if (dim == 1)
		std::sort(objs_list.begin() + left_index, objs_list.begin() + right_index - 1, &TriCompare_Y);
	else
		std::sort(objs_list.begin() + left_index, objs_list.begin() + right_index - 1, &TriCompare_Z);

	// sort from left to right and update split_index
	unsigned int split_index = (left_index + right_index) / 2.0;

	// make two node and update their AABB_Box
	ColCal_BVH_Node left_node = ColCal_BVH_Node(ColCal_Box(), 0, left_index);
	ColCal_Box left_b = ColCal_Box(*objs_list[left_index]);
	for (size_t i = left_index + 1; i < split_index; i++) {
		left_b.Include(*objs_list[i]);
	}
	left_node.setBox(left_b);

	ColCal_BVH_Node right_node = ColCal_BVH_Node(ColCal_Box(), 0, split_index);
	ColCal_Box right_b = ColCal_Box(*objs_list[split_index]);
	for (size_t i = split_index + 1; i < right_index; i++) {
		right_b.Include(*objs_list[i]);
	}
	right_node.setBox(right_b);

	// left node
	Build_Recursive_EqualCounts(left_index, split_index, left_node);
	nodes_list.push_back(left_node);
	node.childs[0] = nodes_list.size() - 1;

	// right node
	Build_Recursive_EqualCounts(split_index, right_index, right_node);
	nodes_list.push_back(right_node);
	node.childs[1] = nodes_list.size() - 1;
}

void ColCal_BVH::Build_Recursive_Middle(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node) {
	if ((right_index - left_index) <= this->maxInterNum) {
		// make leaf
		node.makeLeaf(left_index, right_index);
	}

	// find a largest dimension
	ColCal_DataType Range_X = node.box.Max[0] - node.box.Min[0];
	ColCal_DataType Range_Y = node.box.Max[1] - node.box.Min[1];
	ColCal_DataType Range_Z = node.box.Max[2] - node.box.Min[2];

	int dim = 0;
	ColCal_DataType Range_Max = Range_X;
	if (Range_Y > Range_Max) {
		Range_Max = Range_Y;
		dim = 1;
	}
	if (Range_Z > Range_Max) {
		dim = 2;
	}

	if (dim == 0)
		std::sort(objs_list.begin() + left_index, objs_list.begin() + right_index - 1, &TriCompare_X);
	else if (dim == 1)
		std::sort(objs_list.begin() + left_index, objs_list.begin() + right_index - 1, &TriCompare_Y);
	else
		std::sort(objs_list.begin() + left_index, objs_list.begin() + right_index - 1, &TriCompare_Z);

	ColCal_DataType Middle_Objs_CenterPos = (objs_list[left_index]->Points[0][dim] + objs_list[left_index]->Points[1][dim] + objs_list[left_index]->Points[2][dim]
		+ objs_list[right_index - 1]->Points[0][dim] + objs_list[left_index - 1]->Points[1][dim] + objs_list[left_index - 1]->Points[2][dim]) / 12.0;

	// sort from left to right and update split_index
	unsigned int split_index = left_index;
	for (; split_index < right_index; split_index++) {
		if ((objs_list[split_index]->Points[0][dim] + objs_list[split_index]->Points[1][dim] + objs_list[split_index]->Points[2][dim]) / 3.0 > Middle_Objs_CenterPos)
			break;
	}

	// make two node and update their AABB_Box
	ColCal_BVH_Node left_node = ColCal_BVH_Node(ColCal_Box(), 0, left_index);
	ColCal_Box left_b = ColCal_Box(*objs_list[left_index]);
	for (size_t i = left_index + 1; i < split_index; i++) {
		left_b.Include(*objs_list[i]);
	}
	left_node.setBox(left_b);

	ColCal_BVH_Node right_node = ColCal_BVH_Node(ColCal_Box(), 0, split_index);
	ColCal_Box right_b = ColCal_Box(*objs_list[split_index]);
	for (size_t i = split_index + 1; i < right_index; i++) {
		right_b.Include(*objs_list[i]);
	}
	right_node.setBox(right_b);

	// left node
	Build_Recursive_Middle(left_index, split_index, left_node);
	nodes_list.push_back(left_node);
	node.childs[0] = nodes_list.size() - 1;

	// right node
	Build_Recursive_Middle(split_index, right_index, right_node);
	nodes_list.push_back(right_node);
	node.childs[1] = nodes_list.size() - 1;
}

void ColCal_BVH::Build_Recursive_SAH(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node) {

}

void ColCal_BVH::Build_Recursive_HLBVH(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node) {

}

ColCal_BVH_Node* ColCal_BVH::getLeftChild(const ColCal_BVH_Node& node) {
	unsigned int idx = node.childs[0];
	if (idx && idx < nodes_list.size())
		return &nodes_list[idx];
	return nullptr;
}
ColCal_BVH_Node* ColCal_BVH::getRightChild(const ColCal_BVH_Node& node) {
	unsigned int idx = node.childs[1];
	if (idx && idx < nodes_list.size())
		return &nodes_list[idx];
	return nullptr;
}


void ColCal_BVH_Node::setBox(ColCal_Box& b) {
	this->box = ColCal_Box(b);
}

void ColCal_BVH_Node::makeLeaf(unsigned int left, unsigned right) {
	this->leaf = true;
	this->objs_num = right - left;
}