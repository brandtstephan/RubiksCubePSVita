#pragma once
#include <vectormath.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sceerror.h>

#include <gxm.h>
#include <kernel.h>
#include <ctrl.h>
#include <display.h>
#include <libdbg.h>

#include <libdbgfont.h>
#include <math.h>
#include <vectormath.h>

#include <iostream>
using namespace std;  

typedef struct Vertex{ 
		float position[3];	// Easier to index.
		uint32_t color;		// Data gets expanded to float 4 in vertex shader.
		uint8_t rotaFlag;   // Flag to check whether the current axis needs to be rotated or not.
							// If flag throws 1, means that this point should be rotated, 0 else.
}Vertex;
class Cube{
public:
	Cube();
	void makeCube(Vertex cube[]);
	void makeSmallCubes(Vertex face[],float radius, int flag, int dirStaticFace, int color);
	void makeIndices(uint16_t index[]);
	uint32_t setColors(int color);
	void checkSide(Vertex cube[], int currentAxis, int dir);
	void clearRotFlag(Vertex cube[]);
};