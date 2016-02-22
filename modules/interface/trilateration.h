#ifndef TRILATERATION_H_
#define TRILATERATION_H_

/* Largest nonnegative number still considered zero */
#define   MAXZERO  0.0

typedef struct coordinate coordinate;
struct coordinate {
	float  x;
	float  y;
	float  z;
};

void vdiff(coordinate vector1, coordinate vector2, coordinate *result);

void vsum(coordinate vector1, coordinate vector2, coordinate *result);

void vmul(coordinate vector, float n, coordinate *result) ;

void vadd(coordinate vector, float n, coordinate *result);

void vdiv(coordinate vector, float n, coordinate *result);

float  vnorm( coordinate vector);

float dot( coordinate vector1,  coordinate vector2);

void cross( coordinate vector1,  coordinate vector2, coordinate *result);


int trilateration(coordinate * result1, coordinate * result2,
		coordinate p1, float  r1, coordinate p2,
		float  r2, coordinate p3, float  r3,
		float  maxzero);

struct coordinate getResult(const coordinate result1, const coordinate result2,
		const coordinate oldPosition);

void test(coordinate terms[], coordinate * oldTarget, coordinate * target);

#endif /* TRILATERATION_H_ */

