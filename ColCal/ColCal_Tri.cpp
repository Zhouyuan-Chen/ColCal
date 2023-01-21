#include "ColCal_Tri.h"

ColCal_Tri::ColCal_Tri() {
	this->Points[0] = ColCal_Point(0, 0, 0);
	this->Points[1] = ColCal_Point(0, 0, 0);
	this->Points[2] = ColCal_Point(0, 0, 0);
	this->idx = -1;
}

ColCal_Tri::ColCal_Tri(const ColCal_Point p1, const ColCal_Point p2, const ColCal_Point p3, int Idx) {
	this->Points[0] = p1;
	this->Points[1] = p2;
	this->Points[2] = p3;
	this->idx = Idx;
}
ColCal_Tri::ColCal_Tri(const ColCal_Tri& tri) {
	this->Points[0] = tri.Points[0];
	this->Points[1] = tri.Points[1];
	this->Points[2] = tri.Points[2];
	this->idx = tri.idx;
}

ColCal_Tri& ColCal_Tri::operator=(const ColCal_Tri& tri) {
	this->Points[0] = tri.Points[0];
	this->Points[1] = tri.Points[1];
	this->Points[2] = tri.Points[2];
	this->idx = tri.idx;

	return *this;
}

// reference website: https://www.braynzarsoft.net/viewtutorial/q16390-24-triangle-to-triangle-collision-detection
bool ColCal_Tri::ColCal_Collision(const ColCal_Tri& tri) {
	// let's say we have Plane Ax + By + Cz + D = 0
	// then normal vector will be vec3(A, B, C).normalize()
	// we make point(x, y, z, 1.0) be the first vertex of this triangle
	// then D = -(Ax + By + Cz) = - dot(normal, tri.point[0])

	ColCal_Vec3 V0(this->Points[0].x, this->Points[0].y, this->Points[0].z);
	ColCal_Vec3 V1(this->Points[1].x, this->Points[1].y, this->Points[1].z);
	ColCal_Vec3 V2(this->Points[2].x, this->Points[2].y, this->Points[2].z);

	// for this triangle's normal
	ColCal_Vec3 Vec0 = V1 - V0;
	ColCal_Vec3 Vec1 = V2 - V1;
	ColCal_Vec3 Vec2 = V0 - V2;
	ColCal_Vec3 normal = (Vec0 ^ Vec1).normalize();

	float A, B, C;
	A = normal[0];
	B = normal[1];
	C = normal[2];

	// for this plane's D
	ColCal_Vec3 p = ColCal_Vec3(this->Points[0].x, this->Points[0].y, this->Points[0].z);
	float D = -1.0 * (p * normal);


	// then we can use interpolate technique to solve this collision problem
	// if any two vertices of a triangle(let's say them v1 and v2) intersect
	// with this plane, we have following equations: 
	// (1) v_inter = v1 * t + v2 * (1 - t)
	// (2) Ax + By + Cz + D = 0
	// then we can get the t value:
	// A(v1.x * t + v2.x * (1 - t)) + B(v1.y * t + v2.y * (1 - t)) + C(v1.z * t + v2.z * (1 - t)) + D = 0
	// 
	// in this case, if 0<=t<=1, the triangle intersects with this plane and 
	// therefore, we should do some further tests to verify this validation
	// of the collision, otherwise we return false
	//

	ColCal_Vec3 vertices[3];
	vertices[0] = ColCal_Vec3(tri.Points[0].x, tri.Points[0].y, tri.Points[0].z);
	vertices[1] = ColCal_Vec3(tri.Points[1].x, tri.Points[1].y, tri.Points[1].z);
	vertices[2] = ColCal_Vec3(tri.Points[2].x, tri.Points[2].y, tri.Points[2].z);
	float t;
	
	for (int i = 0; i < 3; i++) {
		int idx1 = i;
		int idx2 = (idx1 + 1) % 3;
		float V0x = vertices[idx1][0];
		float V0y = vertices[idx1][1];
		float V0z = vertices[idx1][2];
		float V1x = vertices[idx2][0];
		float V1y = vertices[idx2][1];
		float V1z = vertices[idx2][2];
		t = -1.0 * ((D + A * V1x + B * V1y + C * V1z)
			/ (A * V0x + B * V0y + C * V0z - A * V1x - B * V1y - C * V1z));
		if (t >= 0 && t <= 1) {
			float inter_x, inter_y, inter_z;
			inter_x = t * V0x + (1 - t) * V1x;
			inter_y = t * V0y + (1 - t) * V1y;
			inter_z = t * V0z + (1 - t) * V1z;
			ColCal_Vec3 inter_p(inter_x, inter_y, inter_z);
			// enterclockwise
			ColCal_Vec3 vec_to_p_0 = (inter_p - V0).normalize();
			ColCal_Vec3 vec_to_p_1 = (inter_p - V2).normalize();
			ColCal_Vec3 vec_to_p_2 = (inter_p - V1).normalize();

			if (normal * (vec_to_p_0 ^ -Vec2) > 0 && normal * (vec_to_p_1 ^ -Vec1) > 0 && normal * (vec_to_p_2 ^ -Vec0) > 0) {
				return true;
			}
		}
		else {
			continue;
		}
	}

	return false;
}