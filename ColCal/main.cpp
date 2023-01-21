#include "ColCal_SaP.h"
#include "ColCal_Math.h"

using namespace std;

int main() {
	vector<ColCal_Box> boxes;
	vector<ColCal_Tri> tris;
	std::srand(std::time(nullptr));

	for (int i = 0; i < 10000; i++) {
		float x = (std::rand() % 10000) / 100;
		float y = (std::rand() % 10000) / 100;
		float z = (std::rand() % 10000) / 100;

		ColCal_Point point1(0.0 + x, 0.0 + y, 0.0 + z);
		ColCal_Point point2(1.0 + x, 1.0 + y, 1.0 + z);
		ColCal_Point point3(1.0 + x, 0.0 + y, 0.0 + z);
		tris.push_back(ColCal_Tri(point1, point2, point3, i));
	}

	ColCal_Point point1__(0.0, 0.0, 0.0);
	ColCal_Point point2__(1.0, 1.0, 1.0);
	ColCal_Point point3__(1.0, 0.0, 0.0);
	tris.push_back(ColCal_Tri(point1__, point2__, point3__, 10000));

	ColCal_SaP_Objects obj1(tris, 1);
	tris.clear();
	for (int i = 0; i < 10000; i++) {
		float x = (std::rand() % 10000) / 100;
		float y = (std::rand() % 10000) / 100;
		float z = (std::rand() % 10000) / 100;

		ColCal_Point point1(0.0 + x, 0.0 + y, 0.0 + z);
		ColCal_Point point2(1.0 + x, 1.0 + y, 1.0 + z);
		ColCal_Point point3(1.0 + x, 0.0 + y, 0.0 + z);
		tris.push_back(ColCal_Tri(point1, point2, point3, i));
	}

	ColCal_Point point1_(3.0, 0.0, 0.0);
	ColCal_Point point2_(3.0, 1.0, 1.0);
	ColCal_Point point3_(3.0, 0.0, 0.0);
	tris.push_back(ColCal_Tri(point1_, point2_, point3_, 10000));

	ColCal_SaP_Objects obj2(tris, 2);

	ColCal_SaP_2_Objs _obj_cal(obj1, obj2);


	auto startTime = std::chrono::high_resolution_clock::now();

	_obj_cal.compute();

	auto endTime = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> fp_ms = endTime - startTime;
	cout << fp_ms.count() << std::endl;

	 cout << _obj_cal.getPairsNum();
	// sort + sweep-first + sweep-second + sweep-third + narrow-calculate = toltal-time
	// 50ms +   <50ms     +    <10ms     +    <5ms     +      ~5ms        =   <120ms
	// use PCA -> <<120ms


	//ColCal_Mat4 mat(
	//	5, 0, 0, 0,
	//	0, 3, 5, 0,
	//	1, 0, 5, 0,
	//	1, 0, 0, 1);
	//ColCal_Mat4 mat2(
	//	2, 0, 0, 0,
	//	2, 1, 0, 0,
	//	2, 0, 1, 0,
	//	0, 0, 0, 1);
	//(mat * mat2).print();

}