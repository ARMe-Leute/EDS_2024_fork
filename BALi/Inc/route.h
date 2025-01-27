/*
 * route.h
 *
 *  Created on: 24.01.2024
 *      Author: Flaemig
 */

#ifndef ROUTE_H_
#define ROUTE_H_

/*
 * Routes to drive as a pair of  {Translation and Turn} Steps
 *  one Full Wheel-Turn are 200 FullSteps * 16 Parts  = 3200
 *  maximum are +-32700
 */
#define routeNumMax 2
#define routeStepMax 5

int16_t routeNum = 0;
int16_t routeStep = 0;
const float rotTol = 50;
const float incRotMax = 6;

int16_t route[routeNumMax][routeStepMax][2] =
{
	{		// route 1
			{ 000, 000 },
			{ 0,-16000 },		// turn left
			{ 00,+8000 },		// turn right
			{ 0, -8000 },		// turn left
			{ 0, +16000 },		// backward
	},
	{		// route 2
			{ 000, 000 },
			{ 0,120 },		// turn left
			{ 00,-600 },		// turn right
			{ 0,1200 },		// turn left
			{ 0,-7200 },		// backward
	},

};


#endif /* ROUTE_H_ */



