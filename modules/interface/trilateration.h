#ifndef TRILATERATION_H_
#define TRILATERATION_H_

/* Largest nonnegative number still considered zero */
#define   MAXZERO  0.0

typedef struct coordinate coordinate;
struct coordinate {
	double x;
	double y;
	double z;
};

/* Return the difference of two vectors, (vector1 - vector2). */
coordinate vdiff( coordinate vector1,  coordinate vector2);

/* Return the sum of two vectors. */
coordinate vsum( coordinate vector1,  coordinate vector2);

/* Multiply vector by a number. */
coordinate vmul( coordinate vector,  double n);

/* Divide vector by a number. */
coordinate vdiv( coordinate vector,  double n);

/* Return the Euclidean norm. */
double vnorm( coordinate vector);

/* Return the dot product of two vectors. */
double dot( coordinate vector1,  coordinate vector2);

/* Replace vector with its cross product with another vector. */
coordinate cross( coordinate vector1,  coordinate vector2);

int trilateration(coordinate *  result1, coordinate *  result2,
		 coordinate p1,  double r1,  coordinate p2,
		 double r2,  coordinate p3,  double r3,
		 double maxzero);

struct coordinate getResult( coordinate result1,  coordinate result2,
		 coordinate oldPosition);

struct coordinate vadd( coordinate vector,  int n);

double computeDistance(struct coordinate a, struct coordinate b);

void test(coordinate terms[], coordinate * oldTarget, coordinate * target);

#endif /* TRILATERATION_H_ */

