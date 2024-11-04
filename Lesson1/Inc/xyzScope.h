/*
 * xyzScope.h
 *
 *  Created on: 03.01.2024
 *      Author: tobia
 */

#ifndef XYZSCOPE_H_
#define XYZSCOPE_H_

#define _pi (float)3.141

extern void XYZ2AlphaBeta(int16_t *XYZ_raw, float *AlphaBeta);
extern uint16_t AlBeScreen(float *AlphaBeta);


#endif /* XYZSCOPE_H_ */
