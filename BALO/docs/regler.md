# Regler [PID and Filter Functions]

This software is licensed based on [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/deed.de).

## Table of Contents

  - [Introduction](#Intro)
  - [Examples](#Examples)
  - [Authors](#Author)


<a id="Intro"></a>
## Introduction

1.  toDo some introductions
2.. 


<a id="Examples"></a>

## Examples

```c

#include <regler.h>

/** This it an Example using two PID  Controler
*/
float PID_PosOut, PID_VeloOut;
int main() 
{
 	struct PIDRegler PID_Pos, PID_Velo;
 	const uint8_t iHold = 5;
 	
	// PI.init(... 	KP, KI, KD)
	PID.init(&PID_Pos, 0.5,    0, 0.2);  // Postion  Controler
	PID.init(&PID_Velo, 0.1, 0.2, 0.1);	 // VelocityControler

	PID_PosOut= PID.run(&PID_Pos, ePos);

   
    StepperSetPos(StepL, PID_PosOut); // function call with  argument PID_PosOut
	
	
    //  copy ot Parameter KP,KI,KD from PID_Pos to PID_new
	struct PIDRegler PID_new;
    PID.set(&PID_Pos, &PID_new);

}
```

<a id=Author"></a>
## Authors
[Prof Flämig](mailto:flaemig@dhbw-stuttgart.de)

