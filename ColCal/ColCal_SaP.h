#ifndef SAP_H
#define SAP_H

#include "ColCal.h"
#include "ColCal_Tri.h"
#include "ColCal_Box.h"

inline bool SaP_BoxCompare_X(const ColCal_Box A, const ColCal_Box B) {
	return A.Min[0] < B.Min[0];
};

inline bool SaP_BoxCompare_Y(const ColCal_Box A, const ColCal_Box B) {
	return A.Min[1] < B.Min[1];
};

inline bool SaP_BoxCompare_Z(const ColCal_Box A, const ColCal_Box B) {
	return A.Min[2] < B.Min[2];
};

// for general SaP question
class ColCal_SaP_Objects {
public:
	ColCal_SaP_Objects() {
		this->idx = -1;
		this->axis = 0;
	}
	ColCal_SaP_Objects(std::vector<ColCal_Tri>& tris, int Idx) {
		for (const ColCal_Tri& tri : tris) {
			this->TriArray.push_back(tri);
			ColCal_Box box = ColCal_Box(tri);
			this->BoxArray.push_back(box);
		}
		this->idx = Idx;
		this->axis = 0;
	}

	// the value of axis can be replaced by PCA method
	inline void SaP_Sort(const int Axis = 0) {
		this->axis = Axis;
		if (this->axis == 0)
			std::sort(this->BoxArray.begin(), this->BoxArray.end(), &SaP_BoxCompare_X);
		else if(this->axis == 0)
			std::sort(this->BoxArray.begin(), this->BoxArray.end(), &SaP_BoxCompare_Y);
		else
			std::sort(this->BoxArray.begin(), this->BoxArray.end(), &SaP_BoxCompare_Z);
	}

	inline bool isCollide() {
		return result.size() > 0;
	}
	inline int getPairsNum() {
		return result.size();
	}
	void compute(); 


	std::vector<ColCal_Tri> TriArray;
	std::vector<ColCal_Box> BoxArray;
	std::vector<ColCal_Tri_Pair> result;

	int axis;
	int idx;
};



// for two facet list SaP calculation
class ColCal_SaP_2_Objs {
public:
	ColCal_SaP_2_Objs(ColCal_SaP_Objects obj1, ColCal_SaP_Objects obj2) {
		this->Obj1 = obj1;
		this->Obj2 = obj2;
		this->axis = 0;
	}

	// the value of axis can be replaced by PCA method
	inline void SaP_Sort(const int Axis = 0) {
		this->axis = Axis;
		if (this->axis == 0) {
			std::sort(this->Obj1.BoxArray.begin(), this->Obj1.BoxArray.end(), &SaP_BoxCompare_X);
			std::sort(this->Obj2.BoxArray.begin(), this->Obj2.BoxArray.end(), &SaP_BoxCompare_X);
		}
		else if (this->axis == 1) {
			std::sort(this->Obj1.BoxArray.begin(), this->Obj1.BoxArray.end(), &SaP_BoxCompare_Y);
			std::sort(this->Obj2.BoxArray.begin(), this->Obj2.BoxArray.end(), &SaP_BoxCompare_Y);
		}
		else {
			std::sort(this->Obj1.BoxArray.begin(), this->Obj1.BoxArray.end(), &SaP_BoxCompare_Z);
			std::sort(this->Obj2.BoxArray.begin(), this->Obj2.BoxArray.end(), &SaP_BoxCompare_Z);
		}
	}
	inline bool isCollide() {
		return result.size() > 0;
	}
	inline int getPairsNum() {
		return result.size();
	}
	void compute();


	ColCal_SaP_Objects Obj1, Obj2;
	std::vector<ColCal_Tri_Pair> result;
	int axis;
};

#endif // !COLCAL_H