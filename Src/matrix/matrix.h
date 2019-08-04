//
// Created by 彭伟浩 on 2018/7/1.
/*
 *
 *
 *
 *
 * */
//

#define dex(matrix, a, b) (matrix.data[a*matrix.c+b])
#define matrix_malloc(a,b) malloc(a*b*sizeof(double))
#define m_size(r,c) (r*c*sizeof(double))

#ifndef MATRIX_MATRIX_H
#define MATRIX_MATRIX_H

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>



typedef struct {
    int r;
    int c;
    double *data;
} matrix;





void matrix_multiply(matrix a, matrix b, matrix result);
void eye(matrix m);
void join_row_matrix(matrix a, matrix b, matrix result);
void rotation_matrix_2D(matrix m,double theta);
void zero_martix(matrix m);
void matrix_init(matrix m,int r,int c,double *ptr );

#endif //MATRIX_MATRIX_H
