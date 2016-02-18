#include <stdio.h>
#include <math.h>

#include "trilateration.h"

/* Return the difference of two vectors, (vector1 - vector2). */
coordinate vdiff( coordinate vector1,  coordinate vector2) {
	coordinate v;
	v.x = vector1.x - vector2.x;
	v.y = vector1.y - vector2.y;
	v.z = vector1.z - vector2.z;
	return v;
}

/* Return the sum of two vectors. */
coordinate vsum( coordinate vector1,  coordinate vector2) {
	coordinate v;
	v.x = vector1.x + vector2.x;
	v.y = vector1.y + vector2.y;
	v.z = vector1.z + vector2.z;
	return v;
}

/* Multiply vector by a number. */
coordinate vmul( coordinate vector,  double n) {
	coordinate v;
	v.x = vector.x * n;
	v.y = vector.y * n;
	v.z = vector.z * n;
	return v;
}

/* Divide vector by a number. */
coordinate vdiv( coordinate vector,  double n) {
	coordinate v;
	v.x = vector.x / n;
	v.y = vector.y / n;
	v.z = vector.z / n;
	return v;
}

/* Return the Euclidean norm. */
double vnorm( coordinate vector) {
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

/* Return the dot product of two vectors. */
double dot( coordinate vector1,  coordinate vector2) {
	return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
}

/* Replace vector with its cross product with another vector. */
coordinate cross( coordinate vector1,  coordinate vector2) {
	coordinate v;
	v.x = vector1.y * vector2.z - vector1.z * vector2.y;
	v.y = vector1.z * vector2.x - vector1.x * vector2.z;
	v.z = vector1.x * vector2.y - vector1.y * vector2.x;
	return v;
}

double computeDistance(struct coordinate a, struct coordinate b) {
	double result = 0.0;
	double x = pow(a.x - b.x, 2);
	double y = pow(a.y - b.y, 2);
	double z = pow(a.z - b.z, 2);
	result = sqrt(x + y + z);
	return result;
}

coordinate vadd(const coordinate vector, const int n) {
	coordinate v;
	v.x = vector.x + n;
	v.y = vector.y + n;
	if (vector.z + n < 0) {
		v.z = vector.z + fabs(n);
	} else {
		v.z = vector.z + n;
	}

	return v;
}

/* Return zero if successful, negative error otherwise.
 * The last parameter is the largest nonnegative number considered zero;
 * it is somewhat analoguous to machine epsilon (but inclusive).
 */
int trilateration(coordinate *  result1, coordinate *  result2,
		 coordinate p1,  double r1,  coordinate p2,
		 double r2,  coordinate p3,  double r3,
		 double maxzero) {
	coordinate ex, ey, ez;
	double h, i, j;

	/* h = |p2 - p1|, ex = (p2 - p1) / |p2 - p1| */
	ex = vdiff(p2, p1);
	h = vnorm(ex);
	if (h <= maxzero) {
		/* p1 and p2 are concentric. */
		return -1;
	}
	ex = vdiv(ex, h);

	/* t1 = p3 - p1, t2 = ex (ex . (p3 - p1)) */
	ey = vdiff(p3, p1);
	i = dot(ex, ey);
	ez = vmul(ex, i);

	/* ey = (t1 - t2), t = |t1 - t2| */
	ey = vdiff(ey, ez);
	j = vnorm(ey);
	if (j > maxzero) {
		/* ey = (t1 - t2) / |t1 - t2| */
		ey = vdiv(ey, j);

		/* j = ey . (p3 - p1) */
		j = dot(ey, vdiff(p3, p1));
	} else
		j = 0.0;

	/* Note: t <= maxzero implies j = 0.0. */
	if (fabs(j) <= maxzero) {
		/* p1, p2 and p3 are colinear. */

		/* Is point p1 + (r1 along the axis) the intersection? */
		ez = vsum(p1, vmul(ex, r1));
		if (fabs(vnorm(vdiff(p2, ez)) - r2) <= maxzero
				&& fabs(vnorm(vdiff(p3, ez)) - r3) <= maxzero) {
			/* Yes, t2 is the only intersection point. */
			if (result1)
				*result1 = ez;
			if (result2)
				*result2 = ez;
			return 0;
		}

		/* Is point p1 - (r1 along the axis) the intersection? */
		ez = vsum(p1, vmul(ex, -r1));
		if (fabs(vnorm(vdiff(p2, ez)) - r2) <= maxzero
				&& fabs(vnorm(vdiff(p3, ez)) - r3) <= maxzero) {
			/* Yes, t2 is the only intersection point. */
			if (result1)
				*result1 = ez;
			if (result2)
				*result2 = ez;
			return 0;
		}

		return -2;
	}

	/* ez = ex x ey */
	ez = cross(ex, ey);

	h = (r1 * r1 - r2 * r2) / (2 * h) + h / 2;
	i = (r1 * r1 - r3 * r3 + i * i) / (2 * j) + j / 2 - h * i / j;
	j = r1 * r1 - h * h - i * i;
	if (j < -maxzero) {
		/* The solution is invalid. */
		return -3;
	} else if (j > 0.0)
		j = sqrt(j);
	else
		j = 0.0;

	/* t2 = p1 + x ex + y ey */
	*result1 = vsum(p1, vmul(ex, h));
	*result1 = vsum(*result1, vmul(ey, i));
	*result2 = *result1;

	/* result1 = p1 + x ex + y ey + z ez */
		*result1 = vsum(*result1, vmul(ez, j));

	/* result1 = p1 + x ex + y ey - z ez */
		*result2 = vsum(*result2, vmul(ez, -j));

	return 0;
}

struct coordinate getResult( coordinate result1,  coordinate result2,
		 coordinate oldPosition) {
	if (result1.z < 0) {
		return result2;
	} else if (result2.z < 0) {
		return result1;
	} else {
		double dist1 = sqrt(
				pow(result1.x - oldPosition.x, 2)
						+ pow(result1.y - oldPosition.y, 2)
						+ pow(result1.z - oldPosition.z, 2));
		double dist2 = sqrt(
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
		*target = vadd(*target, translate);
		terms[0] = vadd(terms[0], translate);
		terms[1] = vadd(terms[1], translate);
		terms[2] = vadd(terms[2], translate);

	}
	else {
		*target = vadd(*target, -translate);
		terms[0] = vadd(terms[0], -translate);
		terms[1] = vadd(terms[1], -translate);
		terms[2] = vadd(terms[2], -translate);
	}
}
