#include "Cube.h"


Cube::Cube(){
}
//Just creates the whole cube structure by adding more faces
//to our cube structure.
//Goes through the array jumping 9*4 steps a time.
void Cube::makeCube(Vertex cube[]){
	float pointToVertex = 0.5f;
	int counter = 1;
	int counterCube = 0;
	for(int i = 0; i < 3; i++){
		for(int j = 1; j >= -1; j -= 2){
			makeSmallCubes(&cube[counterCube], pointToVertex, i, j, counter);
			counter++;
			counterCube += 9*4;
		}
	}

}

// Helper function to create small cubes.
// flag 0: x, 1 :y, 2: z.
// dirStaticFace: 1 : positive, -1 : negative direction of the face that is static
// color: just setting the color of that specific cube
void Cube::makeSmallCubes(Vertex face[], float pointToVertex, int flag, int dirStaticFace, int color){

	
	//Separation between the width of the cube and the small cubes inside
	//to leave a chick margin.
	float spacing = 0.01f;

	//Every small cube is a 3rd of our width
	float width = 2.0f * pointToVertex / 3.0f;

	//Start position gets stored
	//Point vertex is the distance from the middle points to the vertices
	float startX = pointToVertex * -1.0f; 
	float startY = pointToVertex * -1.0f; 

	//Constant is for the face we are working on currently
	//Since its static and doesnt change, we only have to check if we are working on the positive or the negative side of the cube
	float constant = pointToVertex * (float)dirStaticFace;
	int counter = 0;

	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			for(float a = spacing;  a <= 1.0f - spacing; a+= 1.0f - spacing * 2.0f){
				for(float b = spacing; b <= 1.0f - spacing; b+= 1.0f - spacing * 2.0f){
					
					face[counter].position[flag] = constant;
					face[counter].position[(flag + 1) % 3] = startX + width * i + width * a;
					face[counter].position[(flag + 2) % 3] = startY + width * j + width * b;
					face[counter].color = setColors(color);
					face[counter].rotaFlag = 0;
					counter++;
				}
			}
		}
	}

}


//This method basically tells the vertex shader which vertices to bind together
//It calculates triangles and passes those binded points
//On Triangle List
void Cube::makeIndices(uint16_t index[]){
	int size = 9*6*6;
	int counter = 0;
	for(int i = 0; i < size ; i+= 6){
		index[i] = counter;
		index[i+1] = counter + 1;
		index[i+2] = counter + 2;

		index[i+3] = counter + 1;
		index[i+4] = counter + 2;
		index[i+5] = counter + 3;

		counter+= 4;
	}

}


//Plain uint method to set the colors in hexa mode
uint32_t Cube::setColors(int color){
	switch(color){
		case 1: return 0xFFFFFF33;
				//Turquoise 
		case 2: return 0xFF00FF99;
				//Green
		case 3: return 0xFFFFFFFF;
				//White
		case 4: return 0xFF00FFFF;
				//Yellow
		case 5: return 0xFFFF33FF; 
				//Pink
		case 6: return 0xFF0000CC;
				//Red
		default: return 0;
				
	}

}

//Method that checks, whether the current axis needs to be rotated or not
void Cube::checkSide(Vertex cube[], int currentAxis, int dir){
	for(int i = 0; i < 9*4*6; i++){
		//Checking negative direction
		if(cube[i].position[currentAxis] >= (0.5f - (2.0f * 0.5f / 3.0f)) && dir == -1){
			cube[i].rotaFlag = 1;
		//Checking positive direction
		}else if(cube[i].position[currentAxis] <= -(0.5f - (2.0f * 0.5f / 3.0f)) && dir == 1)
			cube[i].rotaFlag = 1;
		//Checking middle column
		else if (cube[i].position[currentAxis] > -(0.5f - (2.0f * 0.5f / 3.0f)) && cube[i].position[currentAxis] < (0.5f - (2.0f * 0.5f / 3.0f)) && dir == 0){
			cube[i].rotaFlag = 1;
		}
	}
}

//Resets the values of the rotation flag
void Cube::clearRotFlag(Vertex cube[]){
	for(int i = 0; i < 9*4*6; i++){
			cube[i].rotaFlag = 0;
	}
}