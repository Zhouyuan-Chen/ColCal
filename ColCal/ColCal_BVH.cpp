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
	// copy tris to a new vector for further sorting operation
	std::vector<ColCal_Tri*> tris_temp;
	std::vector<ColCal_BVH_Node> nodes;

	// create the root node and World_Box for the root node
	ColCal_BVH_Node root_node = ColCal_BVH_Node();
	ColCal_Box World_Box = ColCal_Box();
	int num_objs = 0;

	if (tris_temp.size() != 0) {
		World_Box = ColCal_Box(*tris_temp[0]);
	}
	for (const auto& o : tris_temp) {
		World_Box.Include(*o);
		tris_temp.push_back(o);
		++num_objs;
	}

	// bond the World_Box to the root node
	root_node.setBox(World_Box);
	Build_Recursive(0, tris_temp.size(), &root_node, this->splitMethod);
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

void ColCal_BVH::Build_Recursive_EqualCounts(int left_index, int right_index, ColCal_BVH_Node* node) {

}

void ColCal_BVH::Build_Recursive_Middle(int left_index, int right_index, ColCal_BVH_Node* node) {

}

void ColCal_BVH::Build_Recursive_SAH(int left_index, int right_index, ColCal_BVH_Node* node) {

}

void ColCal_BVH::Build_Recursive_HLBVH(int left_index, int right_index, ColCal_BVH_Node* node) {

}


void ColCal_BVH_Node::setBox(ColCal_Box& b) {
	this->box = ColCal_Box(b);
}

void ColCal_BVH_Node::makeLeaf(unsigned int left, unsigned right) {

}

void ColCal_BVH_Node::makeNode() {

}