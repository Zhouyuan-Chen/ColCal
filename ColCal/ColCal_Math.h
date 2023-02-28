#ifndef COLCAL_MATH_H
#define COLCAL_MATH_H

#include "ColCal.h"

/////////////////////////////////////////////////////////////////
// Note: In this library, our vector and matrix use row-main 
// system such as: VALUE[ROW][COLUMN]
/////////////////////////////////////////////////////////////////

class ColCal_Vec3 {
public:
	ColCal_Vec3() {
		this->value[0] = this->value[1] = this->value[2] = 0.0;
	}
	ColCal_Vec3(ColCal_DataType x) {
		this->value[0] = this->value[1] = this->value[2] = x;
	}
	ColCal_Vec3(ColCal_DataType x, ColCal_DataType y, ColCal_DataType z) {
		this->value[0] = x;
		this->value[1] = y;
		this->value[2] = z;
	}
	ColCal_Vec3(const ColCal_Vec3& vec) {
		this->value[0] = vec.value[0];
		this->value[1] = vec.value[1];
		this->value[2] = vec.value[2];
	}
	ColCal_Vec3& operator=(const ColCal_Vec3& vec) {
		this->value[0] = vec.value[0];
		this->value[1] = vec.value[1];
		this->value[2] = vec.value[2];
		return *this;
	}
	// dot operation
	ColCal_DataType operator*(const ColCal_Vec3& vec) {
		return this->value[0] * vec.value[0] + this->value[1] * vec.value[1] + this->value[2] * vec.value[2];
	}
	// cross operation
	ColCal_Vec3 operator^(const ColCal_Vec3& vec) {
		return ColCal_Vec3(
			this->value[1] * vec.value[2] - this->value[2] * vec.value[1],
			-(this->value[0] * vec.value[2] - this->value[2] * vec.value[0]),
			this->value[0] * vec.value[1] - this->value[1] * vec.value[0]);
	}
	// plus operation
	ColCal_Vec3 operator+(const ColCal_Vec3& vec) {
		return ColCal_Vec3(this->value[0] + vec.value[0], this->value[1] + vec.value[1], this->value[2] + vec.value[2]);
	}
	// minus operation
	ColCal_Vec3 operator-(const ColCal_Vec3& vec) {
		return ColCal_Vec3(this->value[0] - vec.value[0], this->value[1] - vec.value[1], this->value[2] - vec.value[2]);
	}
	// divide operation
	ColCal_Vec3 operator/(const ColCal_DataType& v) {
		return ColCal_Vec3(this->value[0] / v, this->value[1] / v, this->value[2] / v);
	}
	// self minus operation
	ColCal_Vec3 operator-() {
		return ColCal_Vec3(this->value[0] * -1.0, this->value[1] * -1.0, this->value[2] * -1.0);
	}
	// length operation
	ColCal_DataType length() {
		return sqrtf(this->value[0] * this->value[0] + this->value[1] * this->value[1] + this->value[2] * this->value[2]);
	}
	// nomalize operation
	ColCal_Vec3 normalize() {
		ColCal_DataType len = this->length();
		return ColCal_Vec3(this->value[0] / len, this->value[1] / len, this->value[2] / len);
	}
	ColCal_DataType& operator[](int idx) {
		return this->value[idx];
	}
	void swap(ColCal_Vec3& vec3) {
		ColCal_Vec3 temp = *this;
		*this = vec3;
		vec3 = temp;
	}
	void print() {
		std::cout << "(" << value[0] << ", " << value[1] << ", " << value[2] << ")" << std::endl;
	}

	ColCal_DataType value[3];
};

class ColCal_Vec4 {
public:
	ColCal_Vec4() {
		this->value[0] = this->value[1] = this->value[2] = 0.0;
		this->value[3] = 1.0;
	}
	ColCal_Vec4(ColCal_DataType x) {
		this->value[0] = this->value[1] = this->value[2] = x;
		this->value[3] = 1.0;
	}
	ColCal_Vec4(ColCal_DataType x, ColCal_DataType y, ColCal_DataType z, ColCal_DataType r = 1) {
		this->value[0] = x;
		this->value[1] = y;
		this->value[2] = z;
		this->value[3] = r;
	}
	ColCal_Vec4(const ColCal_Vec4& vec) {
		this->value[0] = vec.value[0];
		this->value[1] = vec.value[1];
		this->value[2] = vec.value[2];
		this->value[3] = vec.value[3];
	}
	ColCal_Vec4(const ColCal_Vec3& vec) {
		this->value[0] = vec.value[0];
		this->value[1] = vec.value[1];
		this->value[2] = vec.value[2];
		this->value[3] = 1.0;
	}
	ColCal_DataType& operator[](int idx) {
		return this->value[idx];
	}
	void print() {
		std::cout << "(" << value[0] << ", " << value[1] << ", " << value[2] << ", " << value[3] << ")" << std::endl;
	}
	ColCal_DataType value[4];
};

class ColCal_Mat4 {
public:
	ColCal_Mat4() {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				if (i != j)
					this->value[i][j] = 0.0;
				else
					this->value[i][j] = 1.0;
			}
		}
	}
	ColCal_Mat4(ColCal_DataType x) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				if (i != j)
					this->value[i][j] = 0.0;
				else
					this->value[i][j] = x;
			}
		}
		this->value[3][3] = 1.0;
	}
	ColCal_Mat4(
		ColCal_DataType x1, ColCal_DataType y1, ColCal_DataType z1, ColCal_DataType r1,
		ColCal_DataType x2, ColCal_DataType y2, ColCal_DataType z2, ColCal_DataType r2,
		ColCal_DataType x3, ColCal_DataType y3, ColCal_DataType z3, ColCal_DataType r3,
		ColCal_DataType x4, ColCal_DataType y4, ColCal_DataType z4, ColCal_DataType r4) {
		this->value[0][0] = x1;	this->value[0][1] = y1;	this->value[0][2] = z1;	this->value[0][3] = r1;
		this->value[1][0] = x2;	this->value[1][1] = y2; this->value[1][2] = z2; this->value[1][3] = r2; 
		this->value[2][0] = x3; this->value[2][1] = y3; this->value[2][2] = z3; this->value[2][3] = r2;
		this->value[3][0] = x4; this->value[3][1] = y4;	this->value[3][2] = z4; this->value[3][3] = r4;
	}
	ColCal_Mat4(const ColCal_Mat4& mat) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				this->value[i][j] = mat.value[i][j];
			}
		}
		this->value[3][3] = 1.0;
	}
	ColCal_Mat4& operator=(const ColCal_Mat4& mat) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				this->value[i][j] = mat.value[i][j];
			}
		}
		this->value[3][3] = 1.0;
	}
	// * operation
	ColCal_Mat4 operator*(const ColCal_Mat4& mat) {
		ColCal_Mat4 ret;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				ret.value[i][j] = this->value[i][0] * mat.value[0][j] + this->value[i][1] * mat.value[1][j]
								+ this->value[i][2] * mat.value[2][j] + this->value[i][3] * mat.value[3][j];
			}
		}
		return ret;
	}
	// * operation with vec4
	ColCal_Vec4 operator*(const ColCal_Vec4& vec) {
		ColCal_Vec4 ret;
		for (int i = 0; i < 4; i++) {
			ret.value[i] = this->value[i][0] * vec.value[0] + this->value[i][1] * vec.value[1]
				+ this->value[i][2] * vec.value[2] + this->value[i][3] * vec.value[3];
		}
		return ret;
	}
	ColCal_DataType* operator[](int idx) {
		return &value[idx][0];
	}
	ColCal_Mat4 getEigenMatrix()const {
		// compute eigenmatrix
		//ColCal_Vec3 row0(cov.value[0][0], cov.value[0][1], cov.value[0][2]),
		//	row1(cov.value[0][0], cov.value[0][1], cov.value[0][2]),
		//	row2(cov.value[0][0], cov.value[0][1], cov.value[0][2]);

		//if (!row0[0]) {
		//	if (row1[0]) {
		//		row0.swap(row1);
		//	}
		//	else if (row2[0]) {
		//		row0.swap(row2);
		//	}
		//}

		//if (!row1[1] && row2[1]) {
		//	row1.swap(row2);
		//}

		//if (row0[0]) {
		//	if (row)
		//}
	}
	void print() {
		std::cout << "[ " << this->value[0][0] << ", " << this->value[0][1] << ", " << this->value[0][2]<< ", " << this->value[0][3] << std::endl;
		std::cout << "  " << this->value[1][0] << ", " << this->value[1][1] << ", " << this->value[1][2]<< ", " << this->value[1][3] << std::endl;
		std::cout << "  " << this->value[2][0] << ", " << this->value[2][1] << ", " << this->value[2][2]<< ", " << this->value[2][3] << std::endl;
		std::cout << "  " << this->value[3][0] << ", " << this->value[3][1] << ", " << this->value[3][2]<< ", " << this->value[3][3] << " ]" << std::endl;
	}
	ColCal_DataType value[4][4];
};

#endif