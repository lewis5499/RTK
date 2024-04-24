/*------------------------------------------------------------------------------
* matrix.h : RTK software matrix operations
*
*          Copyright (C) 2024 by H.Z. Liu, All rights reserved.
*
* options : none
*
* references :  [1]"RTK_Structs.h"
*
* version : $Revision: 1.8 $ $Date: 2024 / 04 / 09  20:30:32 $
*
* history : 2023 / 11 / 03   1.0
*           2023 / 11 / 05   1.1
*           2023 / 11 / 06   1.2
*           2023 / 11 / 24   1.3
*           2023 / 12 / 11   1.4
*           2023 / 12 / 14   1.5
*           2023 / 12 / 25   1.6
*			2024 / 03 / 18	 1.7
*			2024 / 04 / 09	 1.8 new
*-----------------------------------------------------------------------------*/
#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <string>

using namespace std;

namespace matrix {

	////////////////////
	// Original codes //
	////////////////////
	// note: matrix stored by row-major order
	void zero(double *p, int r, int c);
	void zero(int *p, int r, int c);
	/* find the biggest element */
	double maxElement(const double *a, const short row, const short col);
	/* find the smallest element */
	double minElement(const double *a, const short row, const short col);
	/* deep copy a vec/matrix */
	void matcpy(double *dst, const double *ori, int length);
	/* perform scalar*mat */
	void matMul_s(const double scalar, const double *oriMat, double *dstMat, int n);
	/* trim a str */
	static string trim(string s);
	/* Display of matrix: String */
	void matPrint(FILE* fp, const double* iMatrix, const unsigned short row, const unsigned short column);
	string matToStr(const double* iMatrix, const unsigned short row, const unsigned short column);		
	string imatToStr(const int* iMatrix, const unsigned short row, const unsigned short column);
	/* Transpose of matrix */
	bool matTran(const double* iMatrix, const unsigned short row, const unsigned short column, double* iNewMatrix);	
	/* Inverse of matrix */
	bool matInv(const double* iMatrix, const unsigned short size, double* iMatrixInv);
	/* Matrix addition */
	bool matAdd(const double* MatrixLin, const double* MatrixRin, double *MatrixResult,	const unsigned short row, const unsigned short column);
	/* Matrix subtraction */
	bool matSub(const double* MatrixLin, const double* MatrixRin, double* MatrixResult,	const unsigned short row, const unsigned short column);
	/* Matrix multiplication*/
	bool matMul(const double* MatrixLin, const unsigned short rowinL, const unsigned short colinL,	
				const double* MatrixRin, const unsigned short rowinR, const unsigned short colinR, double *MatrixResult);
	/* Get block from matrix */
	bool matBlock(const double* MatrixA, const unsigned short rowA, const unsigned short colA,
		const unsigned short p, const unsigned short q,
		const unsigned short m, const unsigned short n, double* MatrixB);
	/* Resize dynamic matrix */
	int matResize(double *&ori, int preSize, int curSize);
	/* LU: Determinant of a square matrix */
	double mat_nn_LU(const double* iMatrix, const unsigned short size);	
	/* Laplace: Determinant of a square matrix */
	double mat_nn_LA(const double* iMatrix, const unsigned short size);	
	/* Gaussian: Determinant of a square matrix */
	double mat_nn_Gauss(const double* iMatrix, const unsigned short size);	

	///////////////////////////////
	// Inherit codes from RTKlib //
	///////////////////////////////
	// note: matrix stored by column-major order (fortran convension)
	/* inner product of vectors */
	double dot(const double *a, const double *b, int n);
	/* euclid norm of vector */
	double norm(const double *a, int n);
	double norm(const double *a, const double *b, int n);
	/* outer product of 3d vectors */
	void cross3(const double *a, const double *b, double *c);
	/* normalize 3d vector */
	int normv3(const double *a, double *b);
	/* matrix shallow copy */
	void matcpy(double *A, const double *B, int n, int m);
	/* new matrix */
	double *mat(int n, int m);
	/* new integer matrix */
	int *imat(int n, int m);
	/* zero matrix */
	double *zeros(int n, int m);
	/* identity matrix */
	double *eye(int n);
	void eye(double *A, int n);
	/* inverse of matrix */
	int matinv(double *A, int n);
	/* Matrix multiplication*/
	void matmul(const char *tr, int n, int k, int m, double alpha,
		const double *A, const double *B, double beta, double *C);
	/* LU decomposition */
	static int ludcmp(double *A, int n, int *indx, double *d);
	/* LU back-substitution */
	static void lubksb(const double *A, int n, const int *indx, double *b);	

	//////////////////////////////////
	// Inherit codes from Reference //
	//////////////////////////////////
	// note: matrix stored by column-major order (fortran convension)
	int MatrixInv(int n, double a[], double b[]);
	bool MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[]);
	void deleteRowAndColumn(int m, int n, int m1, int n1, double M[]);
	void deleteRow(int rows, int rowToDelete, double vector[]);
	void deleteRow2(int rows, int rowToDelete, int vector[][2]);
}
#endif