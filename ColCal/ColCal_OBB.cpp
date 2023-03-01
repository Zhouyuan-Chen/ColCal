#include "ColCal_OBB.h"

ColCal_OBB::~ColCal_OBB() {
	if(this->points)
		delete[] this->points;
	this->points = nullptr;
}

ColCal_OBB::ColCal_OBB() {
	this->points = nullptr;
	this->rotate = ColCal_Mat4();
	this->pos = ColCal_Vec3();
	this->extension = ColCal_Vec3();
}

ColCal_OBB::ColCal_OBB(const std::vector<ColCal_Point*>& pts) {
	this->points = nullptr;
	this->rotate = ColCal_Mat4();
	this->pos = ColCal_Vec3();
	this->extension = ColCal_Vec3();
	build(pts);
}

ColCal_OBB::ColCal_OBB(const std::vector<ColCal_Tri*>& tris, const std::vector<ColCal_Point*>& pts) {
	this->points = nullptr;
	this->rotate = ColCal_Mat4();
	this->pos = ColCal_Vec3();
	this->extension = ColCal_Vec3();
	build(tris, pts);
}

void ColCal_OBB::build(const std::vector<ColCal_Point*>& pts) {
	// firstly, compute covariance between three axises
	ColCal_DataType cxx, cxy, cxz, cyy, cyz, czz;
	cxx = cxy = cxz = cyy = cyz = czz = 0;
	ColCal_Vec3 mean_vec;

	for (const ColCal_Point* p : pts) {
		mean_vec = mean_vec + (ColCal_Vec3(p->x, p->y, p->z) / (pts.size() * 1.0));
	}
	ColCal_Point mp = ColCal_Point(mean_vec[0], mean_vec[1], mean_vec[2]);

	// calculate covariance matrix
	for (const ColCal_Point* p : pts) {
		cxx += p->x * p->x - mp.x * mp.x;
		cxy += p->x * p->y - mp.x * mp.y;
		cxz += p->x * p->z - mp.x * mp.z;
		cyy += p->y * p->y - mp.y * mp.y;
		cyz += p->z * p->y - mp.z * mp.y;
		czz += p->z * p->z - p->z * p->z;
	}

	ColCal_Mat4 covariance_m = ColCal_Mat4(
		cxx, cxy, cxz, 0.0,
		cxy, cyy, cyz, 0.0,
		cxz, cyz, czz, 0.0,
		0.0, 0.0, 0.0, 1.0
	);

	this->Build_with_Covariance(covariance_m, pts);
}

void ColCal_OBB::build(const std::vector<ColCal_Tri*>& tris, const std::vector<ColCal_Point*>& pts) {
	ColCal_DataType cxx, cxy, cxz, cyy, cyz, czz;
	cxx = cxy = cxz = cyy = cyz = czz = 0;
	ColCal_Vec3 mean_p_toltal, mean_p;
	ColCal_DataType_D A, A_toltal = 0;

	for (const ColCal_Tri* tri : tris) {
		ColCal_Vec3 p0(tri->Points[0].x, tri->Points[0].y, tri->Points[0].z);
		ColCal_Vec3 p1(tri->Points[1].x, tri->Points[1].y, tri->Points[1].z);
		ColCal_Vec3 p2(tri->Points[2].x, tri->Points[2].y, tri->Points[2].z);
		mean_p = (p0 + p1 + p2) / 3.0;
		A = ((p1 - p0) ^ ((p2 - p0).normalize())).length() / 2.0;
		A_toltal += A;
		mean_p_toltal = mean_p_toltal + mean_p * A;

		cxx += (9.0 * mean_p[0] * mean_p[0] + p0[0] * p0[0] + p1[0] * p1[0] + p2[0] * p2[0]) * (A / 12.0);
		cxy += (9.0 * mean_p[0] * mean_p[1] + p0[0] * p0[1] + p1[0] * p1[1] + p2[0] * p2[1]) * (A / 12.0);
		cxz += (9.0 * mean_p[0] * mean_p[2] + p0[0] * p0[2] + p1[0] * p1[2] + p2[0] * p2[2]) * (A / 12.0);
		cyy += (9.0 * mean_p[1] * mean_p[1] + p0[1] * p0[1] + p1[1] * p1[1] + p2[1] * p2[1]) * (A / 12.0);
		cyz += (9.0 * mean_p[1] * mean_p[2] + p0[1] * p0[2] + p1[1] * p1[2] + p2[1] * p2[2]) * (A / 12.0);
		czz += (9.0 * mean_p[2] * mean_p[2] + p0[2] * p0[2] + p1[2] * p1[2] + p2[2] * p2[2]) * (A / 12.0);
	}

	mean_p_toltal = mean_p_toltal / A_toltal;
	cxx /= A_toltal;
	cxy /= A_toltal;
	cxz /= A_toltal;
	cyy /= A_toltal;
	cyz /= A_toltal;
	czz /= A_toltal;

	cxx -= mean_p_toltal[0] * mean_p_toltal[0]; 
	cxy -= mean_p_toltal[0] * mean_p_toltal[1]; 
	cxz -= mean_p_toltal[0] * mean_p_toltal[2];
	cyy -= mean_p_toltal[1] * mean_p_toltal[1];
	cyz -= mean_p_toltal[1] * mean_p_toltal[2];
	czz -= mean_p_toltal[2] * mean_p_toltal[2];

	ColCal_Mat4 covariance_m = ColCal_Mat4(
		cxx, cxy, cxz, 0.0,
		cxy, cyy, cyz, 0.0,
		cxz, cyz, czz, 0.0,
		0.0, 0.0, 0.0, 1.0
	);

	this->Build_with_Covariance(covariance_m, pts);
}

void ColCal_OBB::Build_with_Covariance(const ColCal_Mat4& cov_mat, const std::vector<ColCal_Point*>& pts) {
	ColCal_Mat4 eigen_mat = cov_mat.getEigenMatrix();

	ColCal_Vec3 right(eigen_mat[0][0], eigen_mat[1][0], eigen_mat[2][0]);
	ColCal_Vec3 up(eigen_mat[0][1], eigen_mat[1][1], eigen_mat[2][1]);
	ColCal_Vec3 forward(eigen_mat[0][2], eigen_mat[1][2], eigen_mat[2][2]);

	ColCal_Vec3 min_proj(ColCal_Max_Value), max_proj(ColCal_Min_Value);
	ColCal_Vec3 min, max;

	for (int i = 0; i < pts.size(); i++) {
		ColCal_Vec3 temp_p(pts[i]->x, pts[i]->y, pts[i]->z);
		ColCal_Vec3 proj_p(temp_p * right, temp_p * up, temp_p * forward);
		for (int j = 0; j < 3; j++) {
			if (proj_p[j] < min_proj[i]) {
				min_proj[j] = proj_p[j];
				min[j] = temp_p[j];
			}
			else if (proj_p[j] > max_proj[j]) {
				max_proj[j] = proj_p[j];
				max[j] = temp_p[j];
			}
		}
	}
	
	this->pos = (max + min) * 0.5;
	this->extension = (max_proj - min_proj) * 0.5;
	this->rotate = eigen_mat;

	if (points)
		delete[] points;

	points = new ColCal_Point[8];
	points[0] = ColCal_Point(pos - right * extension[0] - up * extension[1] - forward * extension[2]);
	points[0] = ColCal_Point(pos - right * extension[0] - up * extension[1] + forward * extension[2]);
	points[0] = ColCal_Point(pos - right * extension[0] + up * extension[1] - forward * extension[2]);
	points[0] = ColCal_Point(pos - right * extension[0] + up * extension[1] + forward * extension[2]);
	points[0] = ColCal_Point(pos + right * extension[0] - up * extension[1] - forward * extension[2]);
	points[0] = ColCal_Point(pos + right * extension[0] - up * extension[1] + forward * extension[2]);
	points[0] = ColCal_Point(pos + right * extension[0] + up * extension[1] - forward * extension[2]);
	points[0] = ColCal_Point(pos + right * extension[0] + up * extension[1] + forward * extension[2]);
}

bool ColCal_OBB::collide(const ColCal_OBB& obj) {
	// project 8 points to this obb's space to compare max and min value
	ColCal_Point* obj_points = obj.points;

	ColCal_Vec3 right(rotate[0][0], rotate[1][0], rotate[2][0]);
	ColCal_Vec3 up(rotate[0][1], rotate[1][1], rotate[2][1]);
	ColCal_Vec3 forward(rotate[0][2], rotate[1][2], rotate[2][2]);

	ColCal_Vec3 min_proj(ColCal_Max_Value), max_proj(ColCal_Min_Value);
	for (int i = 0; i < 8; i++) {
		ColCal_Vec3 p(this->points[i].x, this->points[i].y, this->points[i].z);
		ColCal_Vec3 p_proj(p * right, p * up, p * forward);
		min_proj[0] = ColCal_Min(min_proj[0], p_proj[0]);
		min_proj[1] = ColCal_Min(min_proj[1], p_proj[1]);
		min_proj[2] = ColCal_Min(min_proj[2], p_proj[2]);
		max_proj[0] = ColCal_Max(max_proj[0], p_proj[0]);
		max_proj[1] = ColCal_Max(max_proj[1], p_proj[1]);
		max_proj[2] = ColCal_Max(max_proj[2], p_proj[2]);
	}

	ColCal_Vec3 min_proj_(ColCal_Max_Value), max_proj_(ColCal_Min_Value);
	for (int i = 0; i < 8; i++) {
		ColCal_Vec3 p(obj_points[i].x, obj_points[i].y, obj_points[i].z);
		ColCal_Vec3 p_proj(p * right, p * up, p * forward);
		min_proj_[0] = ColCal_Min(min_proj_[0], p_proj[0]);
		min_proj_[1] = ColCal_Min(min_proj_[1], p_proj[1]);
		min_proj_[2] = ColCal_Min(min_proj_[2], p_proj[2]);
		max_proj_[0] = ColCal_Max(max_proj_[0], p_proj[0]);
		max_proj_[1] = ColCal_Max(max_proj_[1], p_proj[1]);
		max_proj_[2] = ColCal_Max(max_proj_[2], p_proj[2]);
	}

	int flag = 0;
	for (int i = 0; i < 3; i++) {
		if (max_proj[i] < min_proj_[i])
			return true;
		if (min_proj[i] > max_proj_[i])
			return true;
	}

	return false;
}

ColCal_Point* ColCal_OBB::getBoxPoints() {
	return points;
}

void ColCal_OBB::update() {

}