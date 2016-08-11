#pragma once

/*

implementation of the iterative closest point algorithm
author : luke lincoln @ lukes611@gmail.com

requires: 

*/

#include "../basics/locv3.h"
#include "../basics/R3.h"

namespace Licp
{

	/*
		takes a list of R3 points : [x1,y1,z1,1], [x2,y2,z2,1], [x3,y3,z3,1] ... [xn,yn,zn,1]
		and returns a matrix where each R3 point is a row:

		[x1, y1, z1, 1]
		[x2, y2, z2, 1]
		[x3, y3, z3, 1]
		 .    .  .   .
		 .    .  .   .
		 .    .  .   .
		[xn, yn, zn, 1]
	*/
	Mat asMatRows(vector<ll_R3::R3> & inp); 

}