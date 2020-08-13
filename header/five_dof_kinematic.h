/*
 * five_dof_kinematic.h
 *
 *  Created on: Jul 13, 2019
 *      Author: xuuyann
 */
 
 /*
 * copied from csdn
 * kinematics of five dof open chain mechanism
 */

#ifndef FIVEDOFKINEMATIC_H_
#define FIVEDOFKINEMATIC_H_

#include "../header/MyMatrix.h"
#define PI 3.141592653
typedef struct DH_Node *PtrToDHNode;
struct DH_Node
{
	// theta
	float th1;
	float th2;
	float th3;
	float th4;
	float th5;
	float th6;
	float th7;
	// d
	float d1;
	float d2;
	float d3;
	float d4;
	float d5;
	float d6;
	float d7;
	// a
	float a1;
	float a2;
	float a3;
	float a4;
	float a5;
	float a6;
	float a7;
	// alpha
	float alp1;
	float alp2;
	float alp3;
	float alp4;
	float alp5;
	float alp6;
	float alp7;
};
typedef PtrToDHNode Input_data;

// 初始化ＤＨ参数
void Init_DH(Input_data DH_para);
// 正运动学推导
Matrix five_dof_fkine(Input_data DH_para, float theta[]);
// 逆运动学推导
Matrix five_dof_ikine(Input_data DH_para, Matrix fk_T);


#endif /* HEADER_FIVE_DOF_KINEMATIC_H_ */