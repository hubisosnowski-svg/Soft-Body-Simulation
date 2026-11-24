#include <GL/freeglut.h>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <iostream>

#ifndef __VECTORF__
#define __VECTORF__
#pragma once

class Vectorf {
public:
	GLfloat x, y, z;

	Vectorf() {}
	Vectorf(GLfloat x1, GLfloat y1, GLfloat z1) : x(x1), y(y1), z(z1) {};
	Vectorf(GLfloat* V) : x(V[0]), y(V[1]), z(V[2]) {}
	Vectorf(GLdouble s) : x(s), y(s), z(s) {}

	GLfloat magnitude() { return sqrt(x * x + y * y + z * z); }

	Vectorf operator-(Vectorf v) {
		return { this->x - v.x, this->y - v.y, this->z - v.z };
	}

	Vectorf operator+(Vectorf v) {
		return { this->x + v.x, this->y + v.y, this->z + v.z };
	}

	Vectorf operator+=(Vectorf v) {
		return { this->x += v.x, this->y += v.y, this->z += v.z };
	}

	Vectorf operator/=(GLfloat m) {
		return { this->x /= m, this->y /= m, this->z /= m };
	}

	Vectorf operator*(GLfloat m) {
		return { this->x * m, this->y * m, this->z * m };
	}

	Vectorf operator*=(GLfloat m) {
		return { this->x *= m, this->y *= m, this->z *= m };
	}

	explicit operator const GLfloat* () const {
		const GLfloat s[] = { x, y, z };
		return s;
	}
};

#endif
