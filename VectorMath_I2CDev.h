#ifndef VectorMath_I2CDev_h
#define VectorMath_I2CDev_h

#include "VectorMath.h"
#include "helper_3dmath.h"
/**
 * Vector conversions
 */
inline VectorF toVectorF(const VectorFloat& v) {
	return VectorF(v.x, v.y, v.z);
}
inline VectorF toVectorF(const VectorInt16& v) { // this assumes 16G range
	return VectorF(v.x/1024.0, v.y/1024.0, v.z/1024.0);
}

/**
 * Quaternion conversions
 */
inline QuatF toQuatF(const Quaternion& q) {
	return QuatF(q.w, q.x, q.y, q.z);
}
#endif