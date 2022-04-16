#ifndef VectorMath_I2CDev_h
#define VectorMath_I2CDev_h

#include "VectorMath.h"
#include "helper_3dmath.h"

 // this assumes 16G range
#define I2CDEV_ACCEL_SCALE 9.81/1024.0
/**
 * Vector conversions
 */
inline VectorF toVectorF(const VectorFloat& v) {
	return VectorF(v.x, v.y, v.z);
}
inline VectorF toVectorF(const VectorInt16& v) {
	return VectorF(v.x*I2CDEV_ACCEL_SCALE, v.y*I2CDEV_ACCEL_SCALE, v.z*I2CDEV_ACCEL_SCALE);
}

/**
 * Quaternion conversions
 */
inline QuatF toQuatF(const Quaternion& q) {
	return QuatF(q.w, q.x, q.y, q.z);
}
#endif