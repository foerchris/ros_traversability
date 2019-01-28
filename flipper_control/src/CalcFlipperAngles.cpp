/*
 * CalcFlipperAngles.cpp
 *
 *  Created on: 16.01.2019
 *      Author: chfo
 */
#include "CalcFlipperAngles.h"
#include "math.h"


CalcFlipperAngles::CalcFlipperAngles()
{
	// Roboter paramter
	R = 0.082;
	r = 0.0375;
	//L = 0.22;
	L = 0.2325;
	//d = std::hypot (r, L);
	theta = atan (r / L);
	dThreshold = sqrt(pow(R,2) + pow(L,2) - pow(R-r,2));
	//xLength  = 0.1;
	yLength  = 0.1;
	//yLength  = 0.2575;
	//	yLength = L+r;
	xLength = L+r;
	trackLength = 0.5;
	FlipperTrackLength = 2*(xLength - R) + trackLength;
	TracksBaseLinkDist = 0.275;
}


CalcFlipperAngles::~CalcFlipperAngles()
{

}



