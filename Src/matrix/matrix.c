//
// Created by 彭伟浩 on 2018/7/4.
//

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "matrix.h"

void matrix_init(matrix m,int r,int c,double *ptr ){
	m.r=r;
	m.c=c;
	m.data=ptr;
	zero_martix(m);
}



void matrix_multiply(matrix a, matrix b, matrix result) {

    if(a.c != b.r){
//        fprintf(stderr,"矩阵一的行和矩阵二的列不相等,无法相乘");
        return;
    }
    double sum = 0;
    result.r = a.r;
    result.c = b.c;
    int n = a.c;

    for (int i = 0; i < b.c; ++i) {       //遍历b的列
        for (int j = 0; j < a.r; ++j) {     //遍历a的行
            for (int k = 0; k < n; ++k) {
                sum=sum+dex(a,j,k)*dex(b,k,i);
            }
            dex(result,j,i)=sum;
            sum=0;
        }
    }
}

void eye(matrix m){     //单位矩阵
    for (int i = 0; i < m.r; ++i) {
        for (int j = 0; j < m.c; ++j) {
            if(i == j)
                dex(m,i,j)=1;
            else
                dex(m,i,j)=0;
        }
    }
}

//matrix d_eye(int a){
//    matrix m;
//    m.c=a;
//    m.r=a;
//    m.data=malloc(a*a*sizeof(double));
//    eye(m);
//    return m;
//}

//void matrix_print(matrix m){
//    for (int i = 0; i < m.r; ++i) {
//        for (int j = 0; j < m.c; ++j) {
//            printf("%7.3f",dex(m,i,j));
//        }
//        printf("\n");
//    }
//}

void join_row_matrix(matrix a, matrix b, matrix result){
    if(a.r != b.r){
//        fprintf(stderr,"矩阵一的行和矩阵二的行不相等,无法进行拼接");
        return;
    }

    int j;

    for (int i = 0; i < a.r; ++i) {
        for (j = 0; j < a.c; ++j) {
            dex(result,i,j)=dex(a,i,j);
        }

        for ( ; j < b.c; ++j) {
            dex(result,i,j)=dex(b,i,j-a.c);
        }
    }

}

void rotation_matrix_2D(matrix m,double theta){
    dex(m,0,0)=cos(theta);
    dex(m,1,0)=sin(theta);
    dex(m,0,1)=-sin(theta);
    dex(m,1,1)=cos(theta);
}

void zero_martix(matrix m){
	for(int i=0;i<m.r;i++){
		for(int j=0;j<m.c;j++)
			dex(m,i,j)=0.0;
	}
}

void diagonal_m_inverse(matrix m){
	for(int i=0;i<m.r && i<m.c;i++)
	{
		if(dex(m,i,i)!=0.0)
			dex(m,i,i)=1.0/dex(m,i,i);
		
	}
		
}
