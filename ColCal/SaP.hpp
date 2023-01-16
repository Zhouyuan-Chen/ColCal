#ifndef SAP_H
#define SAP_H

#include "ColCal.hpp"
#include <algorithm>

// for two facet list calculation
class ColCal_SaP2Objs {
public:
	struct SaP_Pairs {
		SaP_Pairs() {}
		ColCal_Tri tri_target;		// list1's tri
		std::vector<ColCal> tris;	// list2's tris
	};

	ColCal_SaP2Objs(ColCal_Objects obj1, ColCal_Objects obj2) {
		this->Obj1 = obj1;
		this->Obj2 = obj2;
	}

	bool SaP_BoxCompare(const ColCal_Box A, const ColCal_Box B) {
		return A.Min[0] < B.Min[0];
	};

	void SaP_Sort(int axis = 0);
	void compute();

	ColCal_Objects Obj1, Obj2;
	std::vector<ColCal_Pair> result;
	int axis;
};

// the value of axis can be replaced by PCA method
void ColCal_SaP2Objs::SaP_Sort(int Axis = 0) {
	this->axis = Axis;
	std::sort(this->Obj1.BoxArray.begin(), this->Obj1.BoxArray.end(), SaP_BoxCompare);
	std::sort(this->Obj2.BoxArray.begin(), this->Obj2.BoxArray.end(), SaP_BoxCompare);
}

void ColCal_SaP2Objs::compute() {
	// initiate the list of box and prepare for sweep and prune
	SaP_Sort();

	// use this variable value to record last collision-possible tri's idx 
	int Obj2_idx_min = 0; 
	
	// let's say we use Obj1 as the main object list, and then we sweep the whole Obj2
	for (ColCal_Box box1 : this->Obj1.BoxArray) {
		for (ColCal_Box box2 : this->Obj2.BoxArray) {
			if (ColCal_Box_Collision(box1, box2)) 
				// if box1 and box2 intersect, directly sweep following two axises
				// else prune this box and continue
			{ 

			}
			
		}
	}
}

#endif // !COLCAL_H
