//add constant from libmd25 use in main
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

/*****************************
 *         PARAMETERS        *
 *****************************/

// At the moment, we keep floats as we are not (yet) in optimization mode

#define TICKS_PER_REVOLUTION 4096 //360    // Nb ticks per wheel revolution
#define WHEEL_DIAM 97               // Diameter of the wheel (mm)
#define DIST_PER_REVOLUTION 210.481 //304.734 // Distance traveled for a full wheel revolution (in mm)
#define TICKS_PER_DEG 89.00//89.06 //4.86          // Nb of diff tick (enc1 - enc2) it takes to rotate 1 deg (old value: 4.7)

#endif
