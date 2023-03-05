#include "ColCal_Tri.h"

ColCal_Tri::ColCal_Tri() {
	this->Points[0] = ColCal_Point(0, 0, 0);
	this->Points[1] = ColCal_Point(0, 0, 0);
	this->Points[2] = ColCal_Point(0, 0, 0);
	this->idx = -1;
}

ColCal_Tri::ColCal_Tri(const ColCal_Point p1, const ColCal_Point p2, const ColCal_Point p3, unsigned int Idx) {
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

ColCal_Point ColCal_Tri::getMidPoint() {
	return ColCal_Point((this->Points[0].x + this->Points[1].x + this->Points[0].x) / 3.0,
		(this->Points[0].y + this->Points[1].y + this->Points[0].y) / 3.0,
		(this->Points[0].z + this->Points[1].z + this->Points[0].z) / 3.0
	);
}

/*

direction: counterwise

			  \  2 /
			   \  /
				\/
				/\v1
			   /  \
			Vec0  Vec1
		1	 /  0   \    3
			/        \
  _ _ _ _ _v0_ Vec2 _v2_ _ _ _ __
		  /            \
	6    /      5       \   4
*/

unsigned int ColCal_Tri::Point_Inside_Triangle(const ColCal_Point& p, bool needDetail) const {
	ColCal_Vec3 inter_p(p.x, p.y, p.z);

	ColCal_Vec3 V0(this->Points[0].x, this->Points[0].y, this->Points[0].z);
	ColCal_Vec3 V1(this->Points[1].x, this->Points[1].y, this->Points[1].z);
	ColCal_Vec3 V2(this->Points[2].x, this->Points[2].y, this->Points[2].z);

	// for this triangle's normal
	ColCal_Vec3 Vec0 = V1 - V0;
	ColCal_Vec3 Vec1 = V2 - V1;
	ColCal_Vec3 Vec2 = V0 - V2;
	ColCal_Vec3 normal = (Vec0 ^ Vec1).normalize();

	// for v0-p v1-p v2-p
	ColCal_Vec3 p_v0 = (inter_p - V0).normalize();
	ColCal_Vec3 p_v1 = (inter_p - V1).normalize();
	ColCal_Vec3 p_v2 = (inter_p - V2).normalize();

	bool p_v0_x_p_v1 = normal * (p_v0 ^ p_v1) > 0;
	bool p_v1_x_p_v2 = normal * (p_v1 ^ p_v2) > 0;
	bool p_v2_x_p_v0 = normal * (p_v2 ^ p_v0) > 0;

	// case 0
	if (!p_v0_x_p_v1 && !p_v0_x_p_v1 && !p_v0_x_p_v1) {
		if (!needDetail)
			return 1;
		return 0;
	}

	if (!needDetail) {
		return 1;
	}

	// case 1
	if (p_v0_x_p_v1 && !p_v0_x_p_v1 && !p_v0_x_p_v1) {
		return 1;
	}

	// case 2
	if (p_v0_x_p_v1 && p_v0_x_p_v1 && !p_v0_x_p_v1) {
		return 2;
	}

	// case 3
	if (!p_v0_x_p_v1 && p_v0_x_p_v1 && !p_v0_x_p_v1) {
		return 3;
	}

	// case 4
	if (!p_v0_x_p_v1 && p_v0_x_p_v1 && p_v0_x_p_v1) {
		return 4;
	}

	// case 5
	if (!p_v0_x_p_v1 && !p_v0_x_p_v1 && p_v0_x_p_v1) {
		return 5;
	}

	// case 6
	if (p_v0_x_p_v1 && !p_v0_x_p_v1 && p_v0_x_p_v1) {
		return 6;
	}
}

bool ColCal_Tri::collide(const ColCal_Tri& tri, bool considerParallel) {
	// for this triangle's three points
	ColCal_Vec3 V0(this->Points[0].x, this->Points[0].y, this->Points[0].z);
	ColCal_Vec3 V1(this->Points[1].x, this->Points[1].y, this->Points[1].z);
	ColCal_Vec3 V2(this->Points[2].x, this->Points[2].y, this->Points[2].z);

	// for this triangle's normal
	ColCal_Vec3 Vec0 = V1 - V0;
	ColCal_Vec3 Vec1 = V2 - V1;
	ColCal_Vec3 Vec2 = V0 - V2;
	ColCal_Vec3 normal = (Vec0 ^ Vec1).normalize();

	// tri's three points
	ColCal_Vec3 vertices[3];
	vertices[0] = ColCal_Vec3(tri.Points[0].x, tri.Points[0].y, tri.Points[0].z);
	vertices[1] = ColCal_Vec3(tri.Points[1].x, tri.Points[1].y, tri.Points[1].z);
	vertices[2] = ColCal_Vec3(tri.Points[2].x, tri.Points[2].y, tri.Points[2].z);

	if (considerParallel) {
		// test if they were on the same plane
		ColCal_Vec3 normal_tri = ((vertices[1] - vertices[0]) ^ (vertices[2] - vertices[1])).normalize();
		if (normal * normal_tri == 1.0) {
			// if it is true, then we compute the collision in a 2D system
			for (int i = 0; i < 3; i++) {
				if (this->Point_Inside_Triangle(tri.Points[i]))
					return true;
			}
			for (int i = 0; i < 3; i++) {
				if (tri.Point_Inside_Triangle(this->Points[i]))
					return true;
			}
		}
	}

	// if they are not on the same plane, then we compute the collision in a 3D system
	// let's say we have Plane Ax + By + Cz + D = 0
	// then normal vector will be vec3(A, B, C).normalize()
	// we make point(x, y, z, 1.0) be the first vertex of this triangle
	// then D = -(Ax + By + Cz) = - dot(normal, tri.point[0])
	ColCal_DataType A, B, C;
	A = normal[0];
	B = normal[1];
	C = normal[2];

	// for this plane's D
	ColCal_Vec3 p = ColCal_Vec3(this->Points[0].x, this->Points[0].y, this->Points[0].z);
	ColCal_DataType D = -1.0 * (p * normal);

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

	ColCal_DataType t;
	
	for (int i = 0; i < 3; i++) {
		int idx1 = i;
		int idx2 = (idx1 + 1) % 3;
		ColCal_DataType V0x = vertices[idx1][0];
		ColCal_DataType V0y = vertices[idx1][1];
		ColCal_DataType V0z = vertices[idx1][2];
		ColCal_DataType V1x = vertices[idx2][0];
		ColCal_DataType V1y = vertices[idx2][1];
		ColCal_DataType V1z = vertices[idx2][2];
		t = -1.0 * ((D + A * V1x + B * V1y + C * V1z)
			/ (A * V0x + B * V0y + C * V0z - A * V1x - B * V1y - C * V1z));
		if (t >= 0 && t <= 1) {
			ColCal_DataType inter_x, inter_y, inter_z;
			inter_x = t * V0x + (1 - t) * V1x;
			inter_y = t * V0y + (1 - t) * V1y;
			inter_z = t * V0z + (1 - t) * V1z;
			if (this->Point_Inside_Triangle(ColCal_Point(inter_x, inter_y, inter_z))) {
				return true;
			}
		}
		else {
			continue;
		}
	}

	return false;
}

bool ColCal_Tri::collide(const ColCal_Tri& tri, ColCal_Mat4& M, bool considerParallel) {
	// for this triangle's three points
	ColCal_Vec3 V0(this->Points[0].x, this->Points[0].y, this->Points[0].z);
	ColCal_Vec3 V1(this->Points[1].x, this->Points[1].y, this->Points[1].z);
	ColCal_Vec3 V2(this->Points[2].x, this->Points[2].y, this->Points[2].z);

	// for this triangle's normal
	ColCal_Vec3 Vec0 = V1 - V0;
	ColCal_Vec3 Vec1 = V2 - V1;
	ColCal_Vec3 Vec2 = V0 - V2;
	ColCal_Vec3 normal = (Vec0 ^ Vec1).normalize();

	// tri's three points
	ColCal_Vec3 vertices[3];
	vertices[0] = ColCal_Vec3(tri.Points[0].x, tri.Points[0].y, tri.Points[0].z);
	vertices[1] = ColCal_Vec3(tri.Points[1].x, tri.Points[1].y, tri.Points[1].z);
	vertices[2] = ColCal_Vec3(tri.Points[2].x, tri.Points[2].y, tri.Points[2].z);

	ColCal_Vec4 v0_m(vertices[0]);
	ColCal_Vec4 v1_m(vertices[1]);
	ColCal_Vec4 v2_m(vertices[2]);

	v0_m = M * v0_m;
	v1_m = M * v1_m;
	v2_m = M * v2_m;

	vertices[0] = ColCal_Vec3(v0_m[0], v0_m[1], v0_m[2]);
	vertices[1] = ColCal_Vec3(v1_m[0], v1_m[1], v1_m[2]);
	vertices[2] = ColCal_Vec3(v2_m[0], v2_m[1], v2_m[2]);

	if (considerParallel) {
		// test if they were on the same plane
		ColCal_Vec3 normal_tri = ((vertices[1] - vertices[0]) ^ (vertices[2] - vertices[1])).normalize();
		if (normal * normal_tri == 1.0) {
			// if it is true, then we compute the collision in a 2D system
			for (int i = 0; i < 3; i++) {
				if (this->Point_Inside_Triangle(tri.Points[i]))
					return true;
			}
			for (int i = 0; i < 3; i++) {
				if (tri.Point_Inside_Triangle(this->Points[i]))
					return true;
			}
		}
	}

	// if they are not on the same plane, then we compute the collision in a 3D system
	// let's say we have Plane Ax + By + Cz + D = 0
	// then normal vector will be vec3(A, B, C).normalize()
	// we make point(x, y, z, 1.0) be the first vertex of this triangle
	// then D = -(Ax + By + Cz) = - dot(normal, tri.point[0])
	ColCal_DataType A, B, C;
	A = normal[0];
	B = normal[1];
	C = normal[2];

	// for this plane's D
	ColCal_Vec3 p = ColCal_Vec3(this->Points[0].x, this->Points[0].y, this->Points[0].z);
	ColCal_DataType D = -1.0 * (p * normal);

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

	ColCal_DataType t;

	for (int i = 0; i < 3; i++) {
		int idx1 = i;
		int idx2 = (idx1 + 1) % 3;
		ColCal_DataType V0x = vertices[idx1][0];
		ColCal_DataType V0y = vertices[idx1][1];
		ColCal_DataType V0z = vertices[idx1][2];
		ColCal_DataType V1x = vertices[idx2][0];
		ColCal_DataType V1y = vertices[idx2][1];
		ColCal_DataType V1z = vertices[idx2][2];
		t = -1.0 * ((D + A * V1x + B * V1y + C * V1z)
			/ (A * V0x + B * V0y + C * V0z - A * V1x - B * V1y - C * V1z));
		if (t >= 0 && t <= 1) {
			ColCal_DataType inter_x, inter_y, inter_z;
			inter_x = t * V0x + (1 - t) * V1x;
			inter_y = t * V0y + (1 - t) * V1y;
			inter_z = t * V0z + (1 - t) * V1z;
			if (this->Point_Inside_Triangle(ColCal_Point(inter_x, inter_y, inter_z))) {
				return true;
			}
		}
		else {
			continue;
		}
	}

	return false;
}

ColCal_DataType ColCal_Tri::tri_dis(const ColCal_Tri& tri, ColCal_Mat4& M) {
	ColCal_DataType distance = ColCal_Max_Value;

	// for this triangle's three points
	ColCal_Vec3 V0(this->Points[0].x, this->Points[0].y, this->Points[0].z);
	ColCal_Vec3 V1(this->Points[1].x, this->Points[1].y, this->Points[1].z);
	ColCal_Vec3 V2(this->Points[2].x, this->Points[2].y, this->Points[2].z);

	// for this triangle's normal
	ColCal_Vec3 Vec0 = (V1 - V0).normalize();
	ColCal_Vec3 Vec1 = (V2 - V1).normalize();
	ColCal_Vec3 Vec2 = (V0 - V2).normalize();
	ColCal_Vec3 normal = (Vec0 ^ Vec1).normalize();

	// tri's three points
	ColCal_Vec3 vertices[3];
	vertices[0] = ColCal_Vec3(tri.Points[0].x, tri.Points[0].y, tri.Points[0].z);
	vertices[1] = ColCal_Vec3(tri.Points[1].x, tri.Points[1].y, tri.Points[1].z);
	vertices[2] = ColCal_Vec3(tri.Points[2].x, tri.Points[2].y, tri.Points[2].z);

	ColCal_Vec4 t2_v0_m(vertices[0]);
	ColCal_Vec4 t2_v1_m(vertices[1]);
	ColCal_Vec4 t2_v2_m(vertices[2]);

	t2_v0_m = M * t2_v0_m;
	t2_v1_m = M * t2_v1_m;
	t2_v2_m = M * t2_v2_m;

	vertices[0] = ColCal_Vec3(t2_v0_m[0], t2_v0_m[1], t2_v0_m[2]);
	vertices[1] = ColCal_Vec3(t2_v1_m[0], t2_v1_m[1], t2_v1_m[2]);
	vertices[2] = ColCal_Vec3(t2_v2_m[0], t2_v2_m[1], t2_v2_m[2]);

/*

direction: counterwise

			  \  2 /
			   \  /
				\/
				/\v1
			   /  \
			Vec0  Vec1
		1	 /  0   \    3
			/        \
  _ _ _ _ _v0_ Vec2 _v2_ _ _ _ __
		  /            \
	6    /      5       \   4
*/

	for (int i = 0; i < 3; i++) {
		ColCal_Vec3 p_inter = vertices[i] - normal * (normal * (vertices[i] - V0));
		ColCal_Vec3 p = vertices[i];

		ColCal_Vec3 p_v0 = p - V0;
		ColCal_Vec3 p_v1 = p - V1;
		ColCal_Vec3 p_v2 = p - V2;

		unsigned int type = this->Point_Inside_Triangle(ColCal_Point(p_inter[0], p_inter[1], p_inter[2]), true);
		switch (type)
		{
		case 0:
			distance = ColCal_Min(distance, ColCal_Fabs((normal * (vertices[i] - V0))));
			break;
		case 1:
			distance = ColCal_Min(distance, p_v0 * Vec0);
			break;
		case 2:
			distance = ColCal_Min(distance, p_v1.length());
			break;
		case 3:
			distance = ColCal_Min(distance, p_v1 * Vec1);
			break;
		case 4:
			distance = ColCal_Min(distance, p_v2.length());
			break;
		case 5:
			distance = ColCal_Min(distance, p_v2 * Vec2);
			break;
		case 6:
			distance = ColCal_Min(distance, p_v0.length());
			break;
		default:
			distance = ColCal_Max_Value;
			break;
		}

	}

	return distance;
}