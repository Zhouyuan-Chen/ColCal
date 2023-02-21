#include "ColCal_KdTree.h"

KdTree_Node::KdTree_Node() {
	this->index = 0;
	this->axis = 0;
	this->objs_num = 0;
	this->split_value = ColCal_Max_Value;
	this->leaf = false;
}

void KdTree_Node::MakeLeaf(unsigned int Index, unsigned int objs_num) {
	this->leaf = true;
	this->index = index;
	this->objs_num = objs_num;
}

void ColCal_KdTree::Build() {

}

void ColCal_KdTree::RecursiveBuild() {

}