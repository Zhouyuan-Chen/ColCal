#include "ColCal_BVH.h"
/*
	void build(const std::vector<Intersectable *> &objects)
	* Create new vector for Intersectable pointer copies
	* Create new vector for the nodes
	* Create Root node
	* worldBox = AABB(); // world bounding box
	* For each intersectable[i] in objects
	– worldBox.include(intersectable[i] bounding box)
	– Objs.push_back(intersectable[i])
	* EndFor
	* Set world bounding box to root node
	* build_recursive(0, Objs.size(), root, 0);
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
	//case SplitMethod::HLBVH:
	//	Build_Recursive_HLBVH(left_index, right_index, node);
	//	break;
	default:
		Build_Recursive_EqualCounts(left_index, right_index, node);
		break;
	}
}

/*
	void build_recursive(int left_index, int right_index, BVHNode *node, int depth)
	* If ((right_index – left_index) <= Threshold || (other termination criteria))
	– Initiate current node as a leaf with primitives from Objs[left_index] to Objs[right_index]
	* Else
	– Split intersectables into left and right by finding a split_index
	* Make sure that neither left nor right is completely empty
	– Calculate bounding boxes of left and right sides
	– Create two new nodes, leftNode and rightNode and assign bounding boxes
	– Initiate current node as an interior node with leftNode and rightNode as children
	– build_recursive(left_index, split_index, leftNode, depth + 1)
	– build_recursive(split_index, right_index, rightNode, depth + 1)
	* EndIf
*/

void ColCal_BVH::Build_Recursive_EqualCounts(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node) {
	if ((right_index - left_index) <= this->maxInterNum) {
		// make leaf
		node.makeLeaf(left_index, right_index);
	}

	// find a largest dimension
	int dim = this->getOptimalAxis(node, left_index, right_index);

	BVH_Sort(left_index, right_index, dim);

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
	node.getChildsIndex()[0] = nodes_list.size() - 1;

	// right node
	Build_Recursive_EqualCounts(split_index, right_index, right_node);
	nodes_list.push_back(right_node);
	node.getChildsIndex()[1] = nodes_list.size() - 1;
}

void ColCal_BVH::Build_Recursive_Middle(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node) {
	if ((right_index - left_index) <= this->maxInterNum) {
		// make leaf
		node.makeLeaf(left_index, right_index);
	}

	// find a largest dimension
	int dim = this->getOptimalAxis(node, left_index, right_index);

	BVH_Sort(left_index, right_index, dim);

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
	node.getChildsIndex()[0] = nodes_list.size() - 1;

	// right node
	Build_Recursive_Middle(split_index, right_index, right_node);
	nodes_list.push_back(right_node);
	node.getChildsIndex()[1] = nodes_list.size() - 1;
}

/*
	c = ct + (S(Bl)/S(Bp)) * nl * ci + (S(Br)/S(Bp)) * nr * ci
	c = estimated cost of traversing p and its children (l, r)
	ct = ~cost of performing one traversal iteration
	ci = ~cost of performing one intersection test
	nl, r = number of elements in child node
	S(Bl, r) = surface area of child node
	S(Bp) = surface area of parent node
*/
void ColCal_BVH::Build_Recursive_SAH(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node) {
	if ((right_index - left_index) <= this->maxInterNum) {
		// make leaf
		node.makeLeaf(left_index, right_index);
	}

	// find a largest dimension
	int dim = this->getOptimalAxis(node, left_index, right_index);

	BVH_Sort(left_index, right_index, dim);

	// sort from left to right and update split_index
	std::vector<ColCal_DataType> SAH_leftnodes_surfaces, SAH_rightnodes_surfaces;
	// left nodes surfaces
	ColCal_Box temp = ColCal_Box(*objs_list[0]);
	for (size_t i = 0; i < objs_list.size() - 1; i++) {
		temp.Include(*objs_list[i]);
		SAH_leftnodes_surfaces.push_back(temp.getSurfaceArea());
	}
	temp = ColCal_Box(*objs_list[objs_list.size() - 1]);
	// right nodes surfaces
	for (size_t i = 0; i < objs_list.size() - 1; i++) {
		temp.Include(*objs_list[objs_list.size() - 1 - i]);
		SAH_rightnodes_surfaces.push_back(temp.getSurfaceArea());
	}
	std::reverse(SAH_rightnodes_surfaces.begin(), SAH_rightnodes_surfaces.end());

	// compute optimal split_index for SAH
	unsigned int split_index = 0;
	unsigned int SAH_optimal_index = split_index;
	ColCal_DataType surface_parent = node.getBox().getSurfaceArea();
	ColCal_DataType SAH_optimal_cost = ColCal_Max_Value;
	unsigned int toltal_num = right_index - left_index;
	for (; split_index <toltal_num - 1; split_index++) {
		ColCal_DataType surface_leftnodes = SAH_leftnodes_surfaces[split_index];
		ColCal_DataType surface_rightnodes = SAH_rightnodes_surfaces[split_index];
		ColCal_DataType num_left_node = split_index;
		ColCal_DataType num_right_node = toltal_num - split_index;
		ColCal_DataType cost = 1.0 + (surface_leftnodes * num_left_node + surface_rightnodes * num_right_node) / surface_parent;

		// update SAH_optimal_index
		if (cost < SAH_optimal_cost) {
			SAH_optimal_cost = cost;
			SAH_optimal_index = split_index;
		}
	}

	split_index = SAH_optimal_index + left_index + 1;

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
	node.getChildsIndex()[0] = nodes_list.size() - 1;
	
	// right node
	Build_Recursive_EqualCounts(split_index, right_index, right_node);
	nodes_list.push_back(right_node);
	node.getChildsIndex()[1] = nodes_list.size() - 1;
}

//void ColCal_BVH::Build_Recursive_HLBVH(unsigned int left_index, unsigned int right_index, ColCal_BVH_Node& node) {
//
//}

ColCal_BVH_Node* ColCal_BVH::getLeftChild(ColCal_BVH_Node& node) {
	unsigned int idx = node.getChildsIndex()[0];
	if (idx && idx < nodes_list.size())
		return &nodes_list[idx];
	return nullptr;
}
ColCal_BVH_Node* ColCal_BVH::getRightChild(ColCal_BVH_Node& node) {
	unsigned int idx = node.getChildsIndex()[1];
	if (idx && idx < nodes_list.size())
		return &nodes_list[idx];
	return nullptr;
}

void ColCal_BVH_Node::setBox(ColCal_Box& b) {
	this->box = ColCal_Box(b);
}

ColCal_Box ColCal_BVH_Node::getBox() const {
	return this->box;
}

int ColCal_BVH::getOptimalAxis(const ColCal_BVH_Node& node, const unsigned int& left_index, const unsigned& right_index) {
	int ret;
	switch (this->axisMethod)
	{
	case BVH_AxisMethod::Difference:
		ret = getMaxDifAxis(node, left_index, right_index);
		break;
	case BVH_AxisMethod::Variance:
		ret = getMaxVarAxis(node, left_index, right_index);
		break;
	default:
		ret = getMaxDifAxis(node, left_index, right_index);
		break;
	}
	return ret;
}

int ColCal_BVH::getMaxVarAxis(const ColCal_BVH_Node& node, const unsigned int& left_index, const unsigned& right_index) {
	ColCal_DataType Var_X, Var_Y, Var_Z;
	ColCal_DataType Mean_X, Mean_Y, Mean_Z;
	Var_X = Var_Y = Var_Z = 0;
	Mean_X = Mean_Y = Mean_Z = 0;

	for (unsigned int i = left_index; i < right_index; i++) {
		ColCal_Point p = objs_list[i]->getMidPoint();
		Var_X += p.x * p.x;
		Var_Y += p.y * p.y;
		Var_Z += p.z * p.z;
		Mean_X += p.x;
		Mean_Y += p.y;
		Mean_Z += p.z;
	}
	Var_X /= (right_index - left_index) * 1.0; // E(X^2)
	Mean_X /= (right_index - left_index) * 1.0; // E(X)
	Mean_X *= Mean_X; // E(X)^2
	Var_X -= Mean_X;

	Var_Y /= (right_index - left_index) * 1.0;
	Mean_Y /= (right_index - left_index) * 1.0;
	Mean_Y *= Mean_Y;
	Var_Y -= Mean_Y;

	Var_Z /= (right_index - left_index) * 1.0;
	Mean_Z /= (right_index - left_index) * 1.0;
	Mean_Z *= Mean_Z;
	Var_Z -= Mean_Z;

	return Var_X > Var_Y ? 0 : (Var_Y > Var_Z ? 1 : 2);
}

int ColCal_BVH::getMaxDifAxis(const ColCal_BVH_Node& node, const unsigned int& left_index, const unsigned& right_index) {
	ColCal_DataType Range_X = node.getBox().getMax(0) - node.getBox().getMin(0);
	ColCal_DataType Range_Y = node.getBox().getMax(1) - node.getBox().getMin(1);
	ColCal_DataType Range_Z = node.getBox().getMax(2) - node.getBox().getMin(2);

	return Range_X > Range_Y ? 0 : (Range_Y > Range_Z ? 1 : 2);;
}

void ColCal_BVH_Node::makeLeaf(unsigned int left, unsigned right) {
	this->leaf = true;
	this->objs_num = right - left;
}

unsigned int* ColCal_BVH_Node::getChildsIndex(){
	return this->childs;
}