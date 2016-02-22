#include <stdio.h>
#include <math.h>

#include "trilateration.h"

/* Return the difference of two vectors, (vector1 - vector2). */
void vdiff(coordinate vector1, coordinate vector2, coordinate *result) {
	result->x = vector1.x - vector2.x;
	result->y = vector1.y - vector2.y;
	result->z = vector1.z - vector2.z;
}

/* Return the sum of two vectors. */
void vsum(coordinate vector1, coordinate vector2, coordinate *result) {
	result->x = vector1.x + vector2.x;
	result->y = vector1.y + vector2.y;
	result->z = vector1.z + vector2.z;
}

/* Multiply vector by a number. */
void vmul(coordinate vector, float n, coordinate *result) {
	result->x = vector.x * n;
	result->y = vector.y * n;
	result->z = vector.z * n;
}
void vadd(coordinate vector, float n, coordinate *result) {
	result->x = vector.x + n;
	result->y = vector.y + n;
	if (vector.z + n < 0) {
		result->z = vector.z + fabs(n);
	} else {
		result->z = vector.z + n;
	}
}

/* Divide vector by a number. */
void vdiv(coordinate vector, float n, coordinate *result) {
	result->x = vector.x / n;
	result->y = vector.y / n;
	result->z = vector.z / n;
}

/* Return the Euclidean norm. */
float  vnorm( coordinate vector) {
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

/* Return the dot product of two vectors. */
float dot( coordinate vector1,  coordinate vector2) {
	return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
}

/* Replace vector with its cross product with another vector. */
void cross( coordinate vector1,  coordinate vector2, coordinate *result) {
	result->x = vector1.y * vector2.z - vector1.z * vector2.y;
	result->y = vector1.z * vector2.x - vector1.x * vector2.z;
	result->z = vector1.x * vector2.y - vector1.y * vector2.x;
}

/* Return zero if successful, negative error otherwise.
 * The last parameter is the largest nonnegative number considered zero;
 * it is somewhat analoguous to machine epsilon (but inclusive).
 */
int trilateration(coordinate * result1, coordinate * result2,
		coordinate p1, float  r1, coordinate p2,
		float  r2, coordinate p3, float  r3,
		float  maxzero) {
	coordinate ex, ey, ez, t1, t2, tmp1, tmp2;
	float  h, i, j, x, y, z, t;

	/* h = |p2 - p1|, ex = (p2 - p1) / |p2 - p1| */
	vdiff(p2, p1, &ex);
	h = vnorm(ex);
	if (h <= maxzero) {
		/* p1 and p2 are concentric. */
		return -1;
	}
	vdiv(ex, h, &ex);

	/* t1 = p3 - p1, t2 = ex (ex . (p3 - p1)) */
	vdiff(p3, p1, &t1);
	i = dot(ex, t1);
	vmul(ex, i, &t2);

	/* ey = (t1 - t2), t = |t1 - t2| */
	vdiff(t1, t2, &ey);
	t = vnorm(ey);
	if (t > maxzero) {
		/* ey = (t1 - t2) / |t1 - t2| */
		vdiv(ey, t, &ey);

		/* j = ey . (p3 - p1) */
		j = dot(ey, t1);
	} else{
		j = 0.0;
	}

	/* Note: t <= maxzero implies j = 0.0. */
	if (fabs(j) <= maxzero) {
		/* p1, p2 and p3 are colinear. */

		/* Is point p1 + (r1 along the axis) the intersection? */
		vmul(ex, r1, &t2);
		vsum(p1, t2, &t2);
		vdiff(p2, t2, &tmp1);
		vdiff(p3, t2, &tmp2);
		if (fabs(vnorm(tmp1) - r2) <= maxzero
				&& fabs(vnorm(tmp2) - r3) <= maxzero) {
			/* Yes, t2 is the only intersection point. */
			if (result1)
				*result1 = t2;
			if (result2)
				*result2 = t2;
			return 0;
		}

		/* Is point p1 - (r1 along the axis) the intersection? */
		vmul(ex, -r1, &tmp1);
		vsum(p1, tmp1, &t2);
		vdiff(p2, t2, &tmp1);
		vdiff(p3, t2, &tmp2);
		if (fabs(vnorm(tmp1) - r2) <= maxzero
				&& fabs(vnorm(tmp2) - r3) <= maxzero) {
			/* Yes, t2 is the only intersection point. */
			if (result1)
				*result1 = t2;
			if (result2)
				*result2 = t2;
			return 0;
		}

		return -2;
	}

	/* ez = ex x ey */
	cross(ex, ey, &ez);

	x = (r1 * r1 - r2 * r2) / (2 * h) + h / 2;
	y = (r1 * r1 - r3 * r3 + i * i) / (2 * j) + j / 2 - x * i / j;
	z = r1 * r1 - x * x - y * y;
	if (z < -maxzero) {
		/* The solution is invalid. */
		return -3;
	} else if (z > 0.0)
		z = sqrt(z);
	else
		z = 0.0;

	/* t2 = p1 + x ex + y ey */
	vmul(ex, x, &tmp1);
	vsum(p1, tmp1, &t2);
	vmul(ey, y, &tmp2);
	vsum(t2, tmp2, &t2);

	/* result1 = p1 + x ex + y ey + z ez */
	if (result1){
		vmul(ez, z, &tmp1);
		vsum(t2, tmp1, result1);
	}
	/* result1 = p1 + x ex + y ey - z ez */
	if (result2){
		vmul(ez, -z, &tmp1);
		vsum(t2, tmp1, result1);
	}
	return 0;
}

struct coordinate getResult(const coordinate result1, const coordinate result2,
		const coordinate oldPosition) {
	if (result1.z < 0) {
		return result2;
	} else if (result2.z < 0) {
		return result1;
	} else {
		float  dist1 = sqrt(
				pow(result1.x - oldPosition.x, 2)
						+ pow(result1.y - oldPosition.y, 2)
						+ pow(result1.z - oldPosition.z, 2));
		float  dist2 = sqrt(
				pow(result2.x - oldPosition.x, 2)
						+ pow(result2.y - oldPosition.y, 2)
						+ pow(result2.z - oldPosition.z, 2));
		if (dist1 > dist2) {
			return result2;
		} else {
			return result1;
		}
	}
}

float computeDistance(struct coordinate a, struct coordinate b) {
	float  result = 0.0;
	float  x = pow(a.x - b.x, 2);
	float  y = pow(a.y - b.y, 2);
	float  z = pow(a.z - b.z, 2);
	result = sqrt(x + y + z);
	return result;
}

void test(coordinate terms[], coordinate * oldTarget, coordinate * target){
	int  k;
	double distance[3];
	struct coordinate o1, o2;
	for (k = 0; k < 3; k++) {
		distance[k] = computeDistance(*target, terms[k]);
	}
	trilateration(&o1, &o2, terms[0], distance[0], terms[1], distance[1],
			terms[2], distance[2], MAXZERO);
	*oldTarget  = getResult(o1, o2, *target);
	int translate = 300;
	if ((translate % 2) == 0) {
		vadd(*target, translate, target);
		vadd(terms[0], translate, &terms[0]);
		vadd(terms[1], translate, &terms[1]);
		vadd(terms[2], translate, &terms[2]);
	}
	else {
		vadd(*target, -translate, target);
		vadd(terms[0], -translate, &terms[0]);
		vadd(terms[1], -translate, &terms[1]);
		vadd(terms[2], -translate, &terms[2]);
	}
}
