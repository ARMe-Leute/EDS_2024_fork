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
 *
 *  for 360Deg Rotation of the Balancer Wheel Diameter = 123mm, Distance between Wheels = 220mm
 *  on full turn take 220/123 * 3200 = 5723
 */
#define routeNumMax 2
#define routeStepMax 5

int16_t routeNum = 0;
int16_t routeStep = 0;
float posTol = 0, rotTol = 50;
//const float incRotMax = 18;

/*
 * Rot
 */


int16_t route[routeNumMax][routeStepMax][2] =
{
	{		// route 1
			{ 000, 000 },
			{ 0,+3050 },		// turn 0.25 left
			{ 0,-6100 },		// turn 0.5 right
			{ 0, +12200 },		// turn 1 left
			{ 0, -9150 },		// backward
	},
	{		// route 2
			{ 000, 000 },
			{ 2000, 3050 },		// turn left
			{ 00, 3050 },		// turn right
			{ 2000,3050 },		// turn left
			{ 0,-9150 },		// backward
	},

};


#endif /* ROUTE_H_ */



