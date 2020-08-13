/*
 * FiveDofKinemate.c
 *
 *  Created on: Jul 13, 2019
 *      Author: xuuyann
 */

/*
* Xu Lu copied from csdn on Aug. 13th 2020
* source code of five_dof_kinematics
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../header/FiveDofKinematic.h"
#include "../header/MyMatrix.h"

/*               theta,   d,     a,    alpha                     */
float DH[7][4] = {{0,     0,     0,     PI/2},
				  {0,     0,     0.104, 0},
				  {0,     0,     0.096, 0},
				  {0,     0,     0,     0},
				  {PI/2,  0,     0,     PI/2},
				  {0,     0,     0,     0},
				  {0,     0.163, 0.028, 0}};

void Init_DH(Input_data DH_para)
{
	DH_para->th1 = DH[0][0];
	DH_para->th2 = DH[1][0];
	DH_para->th3 = DH[2][0];
	DH_para->th4 = DH[3][0];
	DH_para->th5 = DH[4][0];
	DH_para->th6 = DH[5][0];
	DH_para->th7 = DH[6][0];
	DH_para->d1 = DH[0][1];
	DH_para->d2 = DH[1][1];
	DH_para->d3 = DH[2][1];
	DH_para->d4 = DH[3][1];
	DH_para->d5 = DH[4][1];
	DH_para->d6 = DH[5][1];
	DH_para->d7 = DH[6][1];
	DH_para->a1 = DH[0][2];
	DH_para->a2 = DH[1][2];
	DH_para->a3 = DH[2][2];
	DH_para->a4 = DH[3][2];
	DH_para->a5 = DH[4][2];
	DH_para->a6 = DH[5][2];
	DH_para->a7 = DH[6][2];
	DH_para->alp1 = DH[0][3];
	DH_para->alp2 = DH[1][3];
	DH_para->alp3 = DH[2][3];
	DH_para->alp4 = DH[3][3];
	DH_para->alp5 = DH[4][3];
	DH_para->alp6 = DH[5][3];
	DH_para->alp7 = DH[6][3];
}

// 正运动学推导
Matrix five_dof_fkine(Input_data DH_para, float theta[])
{
	Matrix rst, Ti;
	rst = eye(4);
	Ti = Create_Matrix(4, 4);
	float a[7] = {DH_para->a1, DH_para->a2, DH_para->a3, DH_para->a4, DH_para->a5, DH_para->a6, DH_para->a7};
	float d[7] = {DH_para->d1, DH_para->d2, DH_para->d3, DH_para->d4, DH_para->d5, DH_para->d6, DH_para->d7};
	float alp[7] = {DH_para->alp1, DH_para->alp2, DH_para->alp3, DH_para->alp4, DH_para->alp5, DH_para->alp6, DH_para->alp7};
	float th[7] = {theta[0], theta[1], theta[2], theta[3], DH_para->th5, theta[4], DH_para->th7};
	for (int i = 0; i < 7; i++){
		Ti->data[0][0] = cos(th[i]);
		Ti->data[0][1] = -sin(th[i]) * cos(alp[i]);
		Ti->data[0][2] = sin(th[i]) * sin(alp[i]);
		Ti->data[0][3] = a[i] * cos(th[i]);
		Ti->data[1][0] = sin(th[i]);
		Ti->data[1][1] = cos(th[i]) * cos(alp[i]);
		Ti->data[1][2] = -cos(th[i]) * sin(alp[i]);
		Ti->data[1][3] = a[i] * sin(th[i]);
		Ti->data[2][0] = 0;
		Ti->data[2][1] = sin(alp[i]);
		Ti->data[2][2] = cos(alp[i]);
		Ti->data[2][3] = d[i];
		Ti->data[3][0] = 0;
		Ti->data[3][1] = 0;
		Ti->data[3][2] = 0;
		Ti->data[3][3] = 1;
		//Show_Matrix(Ti);
		//printf("\n");
		// Matrix Mult_Matrix(Matrix mat_1, Matrix mat_2);
		rst = Mult_Matrix(rst, Ti);
		//Show_Matrix(rst);
	}
	return rst;
}

Matrix five_dof_ikine(Input_data DH_para, Matrix fk_T)
{
	Matrix ik_T;
	ik_T = Create_Matrix(2, 5);
	float a2 = DH_para->a2;
	float a3 = DH_para->a3;
	float ae = DH_para->a7;
	float de = DH_para->d7;
	float nx, ny, nz;
	float ox, oy, oz;
	float ax, ay, az;
	float px, py, pz;
	nx = PickInMat(fk_T, 1, 1);
	ny = PickInMat(fk_T, 2, 1);
	nz = PickInMat(fk_T, 3, 1);
	ox = PickInMat(fk_T, 1, 2);
	oy = PickInMat(fk_T, 2, 2);
	oz = PickInMat(fk_T, 3, 2);
	ax = PickInMat(fk_T, 1, 3);
	ay = PickInMat(fk_T, 2, 3);
	az = PickInMat(fk_T, 3, 3);
	px = PickInMat(fk_T, 1, 4);
	py = PickInMat(fk_T, 2, 4);
	pz = PickInMat(fk_T, 3, 4);
	// theta1
	float theta1;
	theta1 = atan2(py - ny*ae - ay*de, px - nx*ae - ax*de);
	// theta5;
	float theta5;
	theta5 = atan2(sin(theta1)*nx - cos(theta1)*ny, sin(theta1)*ox - cos(theta1)*oy);
	// theta3
	float m = px - nx*ae - ax*de;
	float n = py - ny*ae - ay*de;
	float t = pz - nz*ae - az*de;
	float c3;
	c3 = (pow(cos(theta1), 2)*pow(m, 2) + pow(sin(theta1), 2)*pow(n, 2)
			+ 2*sin(theta1)*cos(theta1)*m*n + pow(t, 2) - pow(a2, 2) - pow(a3, 2)) / (2*a2*a3);
	float theta3_1 = atan2(sqrt(1-pow(c3, 2)), c3);
	float theta3_2 = atan2(-sqrt(1-pow(c3, 2)), c3);
	// theta2
	float c2_1, s2_1, c2_2, s2_2;
	c2_1 = ((a3*cos(theta3_1) + a2)*(cos(theta1)*m + sin(theta1)*n) + a3*sin(theta3_1)*t)
			/ (pow(a3*cos(theta3_1) + a2, 2) + pow(a3, 2)*pow(sin(theta3_1), 2));
	s2_1 = ((a3*cos(theta3_1) + a2)*t - a3*sin(theta3_1)*(cos(theta1)*m + sin(theta1)*n))
			/ (pow(a3*cos(theta3_1) + a2, 2) + pow(a3, 2)*pow(sin(theta3_1), 2));
	c2_2 = ((a3*cos(theta3_2) + a2)*(cos(theta1)*m + sin(theta1)*n) + a3*sin(theta3_2)*t)
			/ (pow(a3*cos(theta3_2) + a2, 2) + pow(a3, 2)*pow(sin(theta3_2), 2));
	s2_2 = ((a3*cos(theta3_2) + a2)*t - a3*sin(theta3_2)*(cos(theta1)*m + sin(theta1)*n))
			/ (pow(a3*cos(theta3_2) + a2, 2) + pow(a3, 2)*pow(sin(theta3_2), 2));
	float theta2_1 = atan2(s2_1, c2_1);
	float theta2_2 = atan2(s2_2, c2_2);
	// theta4
	float theta4_1 = atan2(az, cos(theta1)*ax + sin(theta1)*ay) - theta3_1 - theta2_1;
	float theta4_2 = atan2(az, cos(theta1)*ax + sin(theta1)*ay) - theta3_2 - theta2_2;

	float th[10] = {theta1, theta2_1, theta3_1, theta4_1, theta5,
					theta1, theta2_2, theta3_2, theta4_2, theta5};
	SetData_Matrix(ik_T, th);

	return ik_T;
}