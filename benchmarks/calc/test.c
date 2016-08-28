#include <omp.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#define W 10000
#define H 10000

#define FLOAT float
#define INT int

typedef FLOAT FA[H][W];
typedef int IA[H][W];

int main()
{
//	omp_set_num_threads(2);

	FA* a = (FA*)malloc(sizeof(FA));
	IA* b = (IA*)malloc(sizeof(IA));
	setbuf(stdout, NULL);
	printf("Float size: %d\n", sizeof((*a)[0][0]));
	printf("Int size: %d\n", sizeof((*b)[0][0]));
	time_t t1, t2;
	t1 = time(NULL);
//	printf("Start at %d\n", t1);
	for(int i=0; i<W; i++) {
#pragma omp parallel for
#pragma omp simd collapse(2)
		for(int j=0; j<H; j++) {
			(*a)[j][i] = j * i;
		}
	}
	t2 = time(NULL);
//	printf("End at %d\n", t2);
	printf("Float: %d\n", t2 - t1);
	t1 = time(NULL);
//	printf("Start at %d\n", t1);
	for(int i=0; i<W; i++) {
#pragma omp parallel for
#pragma omp simd collapse(2)
		for(int j=0; j<H; j++) {
			(*b)[j][i] = j * i;
		}
	}
	t2 = time(NULL);
//	printf("End at %d\n", t2);
	printf("Int: %d\n", t2 - t1);
	free(a);
	free(b);
}
