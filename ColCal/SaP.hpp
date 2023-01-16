#ifndef SAP_H
#define SAP_H

#include "ColCal.hpp"
#include "ColCal_Box.h"
#include <algorithm>


// for general SaP question
class ColCal_SaP_Objects {
public:
	ColCal_SaP_Objects() {
		this->idx = -1;
		this->axis = 0;
	}
	ColCal_SaP_Objects(std::vector<ColCal_Tri> tris, int Idx) {
		for (ColCal_Tri tri : tris) {
			this->TriArray.push_back(tri);
			ColCal_Box box = ColCal_Box(tri);
			this->BoxArray.push_back(box);
		}
		this->idx = Idx;
		this->axis = 0;
	}

	inline bool SaP_BoxCompare(const ColCal_Box A, const ColCal_Box B) {
		return A.Min[this->axis] < B.Min[this->axis];
	};
	// the value of axis can be replaced by PCA method
	void SaP_Sort(const int Axis = 0) {
		this->axis = Axis;
		std::sort(this->BoxArray.begin(), this->BoxArray.end(), SaP_BoxCompare);
	}
	bool isCollide() {
		return result.size() > 0;
	}
	int getPairsNum() {
		return result.size();
	}
	void compute(); 


	std::vector<ColCal_Tri> TriArray;
	std::vector<ColCal_Box> BoxArray;
	std::vector<ColCal_Tri_Pair> result;

	int axis;
	int idx;
};

void ColCal_SaP_Objects::compute() {
	// initiate
	result.clear();

	// let's sweep x-axis first, then y and z
	this->axis = 0;

	// initiate the list of box and prepare for sweep and prune
	SaP_Sort(this->axis);

	// let's say we use idx-n object sweep the whole list 
	// from n+1 to x(where no box intersects with the current box)
	for (int cur_idx = 0; cur_idx < this->BoxArray.size(); cur_idx++) {
		ColCal_Box cur_box = this->BoxArray[cur_idx];
		std::vector<ColCal_Box> cur_box2_set;
		bool has_intersect = false;

		// the first sweep
		for (int i = cur_idx + 1; i < this->BoxArray.size(); i++) {
			ColCal_Box cur_box_cmp = this->BoxArray[i];
			// if no box intersects with current box, then break and sweep the next box in list1
			if (SaP_Passed(cur_box, cur_box_cmp, this->axis)) {
				break;
			}
			if (cur_box.ColCal_Collision_Axis(cur_box_cmp, this->axis)) {
				cur_box2_set.push_back(cur_box_cmp);
			}
		}

		// the second and the third sweep, operate the cur_box_pairs
		int next_axis1 = (this->axis + 1) % 3;
		int next_axis2 = (next_axis1 + 1) % 3;

		ColCal_Tri tri1 = this->TriArray[cur_box.idx];

		for (ColCal_Box box2 : cur_box2_set) {
			// the second sweep
			if (cur_box.ColCal_Collision_Axis(box2, next_axis1)) {
				// the third sweep
				if (cur_box.ColCal_Collision_Axis(box2, next_axis2)) {
					// passed three sweep, it means this collision is valid!
					ColCal_Tri tri2 = this->TriArray[box2.idx];
					// compute two triangles
					if (tri1.ColCal_Collision(tri2)) {
						// if collide, then add to result
						result.push_back(ColCal_Tri_Pair(tri1, tri2));
					}
				}
			}
		}
	}
}


// for two facet list SaP calculation
class ColCal_SaP_2_Objs {
public:
	ColCal_SaP_2_Objs(ColCal_SaP_Objects obj1, ColCal_SaP_Objects obj2) {
		this->Obj1 = obj1;
		this->Obj2 = obj2;
		this->axis = 0;
	}

	inline bool SaP_BoxCompare(const ColCal_Box A, const ColCal_Box B) {
		return A.Min[this->axis] < B.Min[this->axis];
	};

	// the value of axis can be replaced by PCA method
	void SaP_Sort(const int Axis = 0) {
		this->axis = Axis;
		std::sort(this->Obj1.BoxArray.begin(), this->Obj1.BoxArray.end(), SaP_BoxCompare);
		std::sort(this->Obj2.BoxArray.begin(), this->Obj2.BoxArray.end(), SaP_BoxCompare);
	}
	bool isCollide() {
		return result.size() > 0;
	}
	int getPairsNum() {
		return result.size();
	}
	void compute();


	ColCal_SaP_Objects Obj1, Obj2;
	std::vector<ColCal_Tri_Pair> result;
	int axis;
};

// this function is used by SaP, to test whether need to continue to sweep
bool SaP_Passed(const ColCal_Box a, const ColCal_Box b, const int axis = 0) {
	if (a.Max[axis] < b.Min[axis])
		return true;
	return false;
}

void ColCal_SaP_2_Objs::compute() {
	// initiate
	result.clear();

	// let's sweep x-axis first, then y and z
	this->axis = 0;

	// initiate the list of box and prepare for sweep and prune
	SaP_Sort(this->axis);

	// use this variable value to record last collision-possible tri's idx 
	int skip_idx_min = 0; 
	
	// let's say we use Obj1 as the main object list, and then we sweep the whole Obj2
	for (ColCal_Box box1 : this->Obj1.BoxArray) {
		ColCal_Box cur_box = box1;
		std::vector<ColCal_Box> cur_box2_set;
		bool has_intersect = false;

		// the first sweep
		for (int i = skip_idx_min; i < Obj2.BoxArray.size(); i++) {
			ColCal_Box cur_box_cmp = Obj2.BoxArray[i];
			// if no box intersects with current box, then break and sweep the next box in list1
			if (SaP_Passed(cur_box, cur_box_cmp, this->axis)) {
				break;
			}
			if (cur_box.ColCal_Collision_Axis(cur_box_cmp, this->axis)) {
				if (!has_intersect) {
					has_intersect = true;
					// we are only supposed to record the first idx, because the following
					// boxes could still be able to collide with the first idx box in list2
					skip_idx_min = i;
				}
				cur_box2_set.push_back(cur_box_cmp);
			}
			// if no intersection and no continue, then it means there are serveral
			// boxes which in list2 have smaller axis-position compared with current box,
			// therefore, it is not necessary to sweep these boxes again, we record the last
			// idx to skip these boxes in the next sweep
			if (!has_intersect) {
				skip_idx_min = i;
			}
		}

		// the second and the third sweep, operate the cur_box_pairs
		int next_axis1 = (this->axis + 1) % 3;
		int next_axis2 = (next_axis1 + 1) % 3;
		ColCal_Tri tri1 = Obj1.TriArray[cur_box.idx];

		for (ColCal_Box box2 : cur_box2_set) {
			// the second sweep
			if (cur_box.ColCal_Collision_Axis(box2, next_axis1)) {
				// the third sweep
				if (cur_box.ColCal_Collision_Axis(box2, next_axis2)) {
					// passed three sweep, it means this collision is valid!
					ColCal_Tri tri2 = Obj2.TriArray[box2.idx];
					// compute two triangles
					if (tri1.ColCal_Collision(tri2)) {
						// if collide, then add to result
						result.push_back(ColCal_Tri_Pair(tri1, tri2));
					}
				}
			}
		}
	}
}

#endif // !COLCAL_H