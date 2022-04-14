#ifndef VectorMath_h
#define VectorMath_h
/**
VectorMath library for ArdIU
*/

#include <math.h>
#include "Arduino.h"

enum StockVector {
	V_ZEROES,
	V_ONES,
	V_X,
	V_Y,
	V_Z
};

class QuatF; // pre-declare so we can use it in VectorF
class VectorF: public Printable {
  public:
	float x, y, z;
	VectorF(float xval, float yval, float zval);
	VectorF(StockVector v);
	VectorF(QuatF q);
	VectorF();

	size_t printTo(Print&) const;

	float magSq() const;
	float mag() const;
	VectorF scale(float s) const;
	inline VectorF operator * (float s) const { return scale(s); }
	inline VectorF normalize() { return scale(1/mag()); }
	float dot(const VectorF &other) const;
	VectorF cross(const VectorF &other) const;
	VectorF rotate(const QuatF &q) const;
};

class QuatF: public Printable {
  public:
	float w, x, y, z;
	QuatF(float wval, float xval, float yval, float zval);
	QuatF(const VectorF& lv);
	QuatF();
	
	size_t printTo(Print& to) const;
	
	QuatF conj() const;
	inline QuatF operator ~ () { return conj(); }
	QuatF mult(const QuatF& other) const;
	inline QuatF operator * (const QuatF& other) const { return mult(other); }
};

#endif