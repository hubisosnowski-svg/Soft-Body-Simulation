#include <GL/freeglut.h>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <iostream>

#ifndef __VECTORD__
#define __VECTORD__
#pragma once

class Vectord {
public:
	GLdouble x, y, z;

	Vectord() {}
	Vectord(GLdouble x1, GLdouble y1, GLdouble z1) : x(x1), y(y1), z(z1) {};
	Vectord(GLdouble* V) : x(V[0]), y(V[1]), z(V[2]) {}
	Vectord(GLdouble s) : x(s), y(s), z(s) {}

	GLdouble magnitude() { return sqrt(x * x + y * y + z * z); }

	Vectord operator-(Vectord v) {
		return { this->x - v.x, this->y - v.y, this->z - v.z };
	}

	Vectord operator+(Vectord v) {
		return { this->x + v.x, this->y + v.y, this->z + v.z };
	}

	Vectord operator+=(Vectord v) {
		return { this->x += v.x, this->y += v.y, this->z += v.z };
	}

	Vectord operator/=(GLdouble m) {
		return { this->x /= m, this->y /= m, this->z /= m };
	}

	Vectord operator*(GLdouble m) {
		return { this->x * m, this->y * m, this->z * m };
	}

	Vectord operator*=(GLdouble m) {
		return { this->x *= m, this->y *= m, this->z *= m };
	}

	explicit operator const GLdouble* () const {
		const GLdouble s[] = { x, y, z };
		return s;
	}
};

#endif