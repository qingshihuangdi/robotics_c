/*
 * MyMatrix.h
 *
 *  Created on: Jul 13, 2019
 *      Author: xuuyann
 */
 
 /*
 * Lu copied from https://blog.csdn.net/qq_26565435/article/details/95792841 on Aug. 13th 2020
 * Matrix calculation for robotics
 */

#ifndef HEADER_MYMATRIX_H_
#define HEADER_MYMATRIX_H_

typedef struct MNode *PtrToMNode;
struct MNode
{
	int row;
	int column;
	float **data;
};
typedef PtrToMNode Matrix;
// 创建一个矩阵
Matrix Create_Matrix(int row, int column);

// 初始化矩阵
void Init_Matrix(Matrix mat);

// 给矩阵每个元素赋值
void SetData_Matrix(Matrix mat, float data[]);

// 打印矩阵
void Show_Matrix(Matrix mat);

// 矩阵加减法
Matrix AddorSub_Matrix(Matrix mat_1, Matrix mat_2, int flag);

// 转置
Matrix Trans_Matrix(Matrix mat);

// 矩阵乘法
Matrix Mult_Matrix(Matrix mat_1, Matrix mat_2);

// 单位矩阵
Matrix eye(int n);

// 取出矩阵某行某列的元素
float PickInMat(Matrix mat, int r, int c);


#endif /* HEADER_MYMATRIX_H_ */