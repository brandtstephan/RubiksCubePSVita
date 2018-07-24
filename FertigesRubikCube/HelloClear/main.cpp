/* SCE CONFIDENTIAL
 * PlayStation(R)Vita Programmer Tool Runtime Library Release 02.000.081
 * Copyright (C) 2010 Sony Computer Entertainment Inc.
 * All Rights Reserved.
 */

// All fonts relöated stuff hs been ripped out.

/*	

	This sample shows how to initialize libdbgfont (and libgxm),
	and render debug font with triangle for clear the screen.

	This sample is split into the following sections:

		1. Initialize libdbgfont
		2. Initialize libgxm
		3. Allocate display buffers, set up the display queue
		4. Create a shader patcher and register programs
		5. Create the programs and data for the clear
		6. Start the main loop
			7. Update step
			8. Rendering step
			9. Flip operation and render debug font at display callback
		10. Wait for rendering to complete
		11. Destroy the programs and data for the clear triangle
		12. Finalize libgxm
		13. Finalize libdbgfont

	Please refer to the individual comment blocks for details of each section.
*/

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sceerror.h>

#include <libsysmodule.h>
#include <razor_capture.h>

#include <gxm.h>
#include <kernel.h>
#include <ctrl.h>
#include <display.h>
#include <libdbg.h>

#include <libdbgfont.h>
#include <math.h>

#include <vectormath.h>
#include "Cube.h"
#include <libsysmodule.h>
#include <razor_capture.h>

#include <touch.h>
using namespace sce::Vectormath::Simd::Aos;


/*	Define the debug font pixel color format to render to. */
#define DBGFONT_PIXEL_FORMAT		SCE_DBGFONT_PIXELFORMAT_A8B8G8R8


/*	Define the width and height to render at the native resolution */
#define DISPLAY_WIDTH				960
#define DISPLAY_HEIGHT				544
#define DISPLAY_STRIDE_IN_PIXELS	1024

/*	Define the libgxm color format to render to.
	This should be kept in sync with the display format to use with the SceDisplay library.
*/
#define DISPLAY_COLOR_FORMAT		SCE_GXM_COLOR_FORMAT_A8B8G8R8
#define DISPLAY_PIXEL_FORMAT		SCE_DISPLAY_PIXELFORMAT_A8B8G8R8

/*	Define the number of back buffers to use with this sample.  Most applications
	should use a value of 2 (double buffering) or 3 (triple buffering).
*/
#define DISPLAY_BUFFER_COUNT		3

/*	Define the maximum number of queued swaps that the display queue will allow.
	This limits the number of frames that the CPU can get ahead of the GPU,
	and is independent of the actual number of back buffers.  The display
	queue will block during sceGxmDisplayQueueAddEntry if this number of swaps
	have already been queued.
*/
#define DISPLAY_MAX_PENDING_SWAPS	2


/*	Helper macro to align a value */
#define ALIGN(x, a)					(((x) + ((a) - 1)) & ~((a) - 1))


/*	The build process for the sample embeds the shader programs directly into the
	executable using the symbols below.  This is purely for convenience, it is
	equivalent to simply load the binary file into memory and cast the contents
	to type SceGxmProgram.
*/
extern const SceGxmProgram binaryClearVGxpStart;
extern const SceGxmProgram binaryClearFGxpStart;

/*	Data structure for clear geometry */
typedef struct ClearVertex
{
	float x;
	float y;
} ClearVertex;


// !! Data related to rendering vertex.
extern const SceGxmProgram binaryBasicVGxpStart;
extern const SceGxmProgram binaryBasicFGxpStart;


/*	Data structure to pass through the display queue.  This structure is
	serialized during sceGxmDisplayQueueAddEntry, and is used to pass
	arbitrary data to the display callback function, called from an internal
	thread once the back buffer is ready to be displayed.

	In this example, we only need to pass the base address of the buffer.
*/
typedef struct DisplayData
{
	void *address;
} DisplayData;

static SceGxmContextParams		s_contextParams;			/* libgxm context parameter */
static SceGxmRenderTargetParams s_renderTargetParams;		/* libgxm render target parameter */
static SceGxmContext			*s_context			= NULL;	/* libgxm context */
static SceGxmRenderTarget		*s_renderTarget		= NULL;	/* libgxm render target */
static SceGxmShaderPatcher		*s_shaderPatcher	= NULL;	/* libgxm shader patcher */

/*	display data */
static void							*s_displayBufferData[ DISPLAY_BUFFER_COUNT ];
static SceGxmSyncObject				*s_displayBufferSync[ DISPLAY_BUFFER_COUNT ];
static int32_t						s_displayBufferUId[ DISPLAY_BUFFER_COUNT ];
static SceGxmColorSurface			s_displaySurface[ DISPLAY_BUFFER_COUNT ];
static uint32_t						s_displayFrontBufferIndex = 0;
static uint32_t						s_displayBackBufferIndex = 0;
static SceGxmDepthStencilSurface	s_depthSurface;

/*	shader data */
static int32_t					s_clearVerticesUId;
static int32_t					s_clearIndicesUId;
static SceGxmShaderPatcherId	s_clearVertexProgramId;
static SceGxmShaderPatcherId	s_clearFragmentProgramId;
// !! Shader pactcher addded.
static SceGxmShaderPatcherId	s_basicVertexProgramId;
static SceGxmShaderPatcherId	s_basicFragmentProgramId;
static SceUID					s_patcherFragmentUsseUId;
static SceUID					s_patcherVertexUsseUId;
static SceUID					s_patcherBufferUId;
static SceUID					s_depthBufferUId;
static SceUID					s_vdmRingBufferUId;
static SceUID					s_vertexRingBufferUId;
static SceUID					s_fragmentRingBufferUId;
static SceUID					s_fragmentUsseRingBufferUId;
static ClearVertex				*s_clearVertices			= NULL;
static uint16_t					*s_clearIndices				= NULL;
static SceGxmVertexProgram		*s_clearVertexProgram		= NULL;
static SceGxmFragmentProgram	*s_clearFragmentProgram		= NULL;
// !! Data added.
static SceGxmVertexProgram		*s_basicVertexProgram		= NULL;
static SceGxmFragmentProgram	*s_basicFragmentProgram		= NULL;


static Vertex					*s_basicVertices			= NULL;

static uint16_t					*s_basicIndices				= NULL;
static int32_t					s_basicVerticesUId;
static int32_t					s_basicIndiceUId;


//!! The program parameter for the transformation of the triangle
static Matrix4 s_finalTransformation;
static const SceGxmProgramParameter *s_wvpParam = NULL;

//This matrix is the new transformation that'd be send to the vertex shader together with our cube,
//To set the new position of the points of our array of vertices
static Matrix4 s_finalColumnRotation;
static const SceGxmProgramParameter *s_rotParam = NULL;


/* Callback function to allocate memory for the shader patcher */
static void *patcherHostAlloc( void *userData, uint32_t size );

/* Callback function to allocate memory for the shader patcher */
static void patcherHostFree( void *userData, void *mem );

/*	Callback function for displaying a buffer */
static void displayCallback( const void *callbackData );

/*	Helper function to allocate memory and map it for the GPU */
static void *graphicsAlloc( SceKernelMemBlockType type, uint32_t size, uint32_t alignment, uint32_t attribs, SceUID *uid );

/*	Helper function to free memory mapped to the GPU */
static void graphicsFree( SceUID uid );

/* Helper function to allocate memory and map it as vertex USSE code for the GPU */
static void *vertexUsseAlloc( uint32_t size, SceUID *uid, uint32_t *usseOffset );

/* Helper function to free memory mapped as vertex USSE code for the GPU */
static void vertexUsseFree( SceUID uid );

/* Helper function to allocate memory and map it as fragment USSE code for the GPU */
static void *fragmentUsseAlloc( uint32_t size, SceUID *uid, uint32_t *usseOffset );

/* Helper function to free memory mapped as fragment USSE code for the GPU */
static void fragmentUsseFree( SceUID uid );


/*	@brief Main entry point for the application
	@return Error code result of processing during execution: <c> SCE_OK </c> on success,
	or another code depending upon the error
*/
int main( void );


// !! Here we create the matrix.
void Update(void);
// !! Here we check which side needs to be rotated and create a rotation matrix to give as value
Matrix4 checkRotatingSide(float angle);
// !! Here we save the rotated points and colors
void setRotation();
// !! Here we set the target angle, requires to find the nearest angle to our current position
float target();
// !! Here we just find the nearest pi to our current angle to set our column on that position "Smoothly"
float searchNearestPi(float pi);
// !! Here we handle all the control inputs
void controlsHandeling();
// !! Here we handle all the touch panel inputs(With a pseudo mundane front touch transformation input)
void touchHandeling();

/*	@brief Initializes the graphics services and the libgxm graphics library
	@return Error code result of processing during execution: <c> SCE_OK </c> on success,
	or another code depending upon the error
*/
static int initGxm( void );

/*	 @brief Creates scenes with libgxm */
static void createGxmData( void );


/*	@brief Main rendering function to draw graphics to the display */
static void render( void );

/*	@brief render libgxm scenes */
static void renderGxm( void );



/*	@brief cycle display buffer */
static void cycleDisplayBuffers( void );

/*	@brief Destroy scenes with libgxm */
static void destroyGxmData( void );




/*	@brief Function to shut down libgxm and the graphics display services
	@return Error code result of processing during execution: <c> SCE_OK </c> on success,
	or another code depending upon the error
*/
static int shutdownGxm( void );



/*	@brief User main thread parameters */
extern const char			sceUserMainThreadName[]		= "simple_main_thr";
extern const int			sceUserMainThreadPriority	= SCE_KERNEL_DEFAULT_PRIORITY_USER;
extern const unsigned int	sceUserMainThreadStackSize	= SCE_KERNEL_STACK_SIZE_DEFAULT_USER_MAIN;

/*	@brief libc parameters */
unsigned int	sceLibcHeapSize	= 1*1024*1024;
/* @Creating my cube*/
static Cube cube;

/* Main entry point of program */
int main( void )
{
	//Turning rear touch pannel on
	sceTouchSetSamplingState(SCE_TOUCH_PORT_BACK,SCE_TOUCH_SAMPLING_STATE_START);

	
	//Turning front touch pannel on
	sceTouchSetSamplingState(SCE_TOUCH_PORT_FRONT,SCE_TOUCH_SAMPLING_STATE_START);

	sceSysmoduleLoadModule(SCE_SYSMODULE_RAZOR_CAPTURE);
	cube = Cube();
	
	int returnCode = SCE_OK;

	/* initialize libdbgfont and libgxm */
	returnCode =initGxm();
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

    SceDbgFontConfig config;
	memset( &config, 0, sizeof(SceDbgFontConfig) );
	config.fontSize = SCE_DBGFONT_FONTSIZE_LARGE;

	returnCode = sceDbgFontInit( &config );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );
	
	/* Message for SDK sample auto test */
	printf( "#### simple: INIT SUCCEEDED ####\n" );


	
	/* create gxm graphics data */
	createGxmData();

	
     // Set sampling mode for input device.
    sceCtrlSetSamplingMode(SCE_CTRL_MODE_DIGITALANALOG_WIDE);

	/* 6. main loop */
	while ( true)
	{
        Update();
		render();
        sceDbgFontPrint( 20, 20, 0xffffffff, "Stephano's rubiks cube" );
		cycleDisplayBuffers();
	}


    /*
	// 10. wait until rendering is done 
	sceGxmFinish( s_context );

	// destroy gxm graphics data 
	destroyGxmData();

	// shutdown libdbgfont and libgxm 
	returnCode = shutdownGxm();
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

	// Message for SDK sample auto test
	printf( "## api_libdbgfont/simple: FINISHED ##\n" );

	return returnCode;
    */
}




float makeFloat(unsigned char input)
{
    return (((float)(input)) / 255.0f * 2.0f) - 1.0f;
}

//Accumulated angle for the cubes transformation
static float s_accumulatedTurningAngleX;
static float s_accumulatedTurningAngleY;

//Accumulated angle responsible for the rows/columns rotation
static float s_accumulatedRotatingAngle;

//Which column/row will be rotated next
static int nextRotPoint = 0;

//Direction of given column, between -1,0,1
static int dir = 1;

//Flags to smoothly allow buttons to be pressed
//But not to abuse given button
static bool squarePressed = false;
static bool trianglePressed = false;
static bool circlePressed = false;
static bool rightPressed = false;
static bool leftPressed = false;

//This flag is specific for both joysticks
static bool isPressed = false;

//Touch flags
static bool touchFlagBack = false;
static bool touchFlagFront= false;


//Flag to allow a smooth transition when rotating a column/row
static bool smoothRotationFlag = false;

//We start our quaternion as an identity
Quat s_accumulatedQuaternion = Quat::identity();

static double factor = 0.5;
void Update (void)
{
	 controlsHandeling();
	 touchHandeling();

	 //To check which sides need to be rotated
	 cube.checkSide(s_basicVertices, (nextRotPoint % 3), dir);

	 //The angle for out quaternion
	 Quat quatAngle = Quat::Quat(s_accumulatedTurningAngleY, s_accumulatedTurningAngleX * -1, 0, 0);
	 //Integration des Quaternions
	 s_accumulatedQuaternion = s_accumulatedQuaternion + 0.06 * ( factor* quatAngle * s_accumulatedQuaternion);
	 //Normalize the Quaternion
	 s_accumulatedQuaternion = normalize(s_accumulatedQuaternion);
	 
	 //Just the overall cube rotation
     Matrix4 rotation = Matrix4::rotation(s_accumulatedQuaternion);
     Matrix4 lookAt = Matrix4::lookAt(Point3(0.0f, 0.0f, -3.0f), Point3(0.0f, 0.0f, 0.0f), Vector3(0.0f, -1.0f, 0.0f));
     Matrix4 perspective = Matrix4::perspective(3.141592f / 4.0f,
		(float)DISPLAY_WIDTH/(float)DISPLAY_HEIGHT,
		0.1f,
		10.0f);
	 //Just to generally do a transformation of the cube as a whole
     s_finalTransformation = perspective * lookAt * rotation;
	 
	
	 //We give to the vertex shader a rotation back in a given axis
	 s_finalColumnRotation = checkRotatingSide(s_accumulatedRotatingAngle);

	 
};

Matrix4 checkRotatingSide(float angle){
	//With this method we want to check which side will be rotating
	//And then we give back, for that side, a rotation matrix
	//For the vertex shader
	Matrix4 rot;
	
	//nextRotPoint works here as a direction helper
	//Example: 0-3-6 are the directions -1,0-1 of a given face, in this case would be Z
	//And so on which each face and its direction
	if(nextRotPoint == 0 || nextRotPoint == 3 || nextRotPoint == 6){
		rot = Matrix4::rotationZYX(Vector3(angle,0.0f , 0.0f));
	}else if(nextRotPoint == 1 || nextRotPoint == 4 || nextRotPoint == 7){
		rot = Matrix4::rotationZYX(Vector3(0.0f,angle, 0.0f));
	}else if(nextRotPoint == 2 || nextRotPoint == 5 || nextRotPoint == 8){
		rot = Matrix4::rotationZYX(Vector3(0.0f,0.0f, angle));
	}

	//Then we proceed to return a rotated point as rotation matrix
	return rot;
}

void setRotation(){

	//Caculate nearest target angle
	float tg = target();

	//we make a rotationg matrix with the angle, where the nearest pi is
	Matrix4 rotMat = checkRotatingSide(tg);
	
	//We check throught our whole cube vertex list
	for(int i = 0; i < 9*4*6;i++){

		//If we find a vertice, where the rotation flag active is
		//Also, meaning that a rotation will happen
		if(s_basicVertices[i].rotaFlag == 1){

			//We proceed to save that point into a vector 4
			Vector4 points = Vector4(s_basicVertices[i].position[0], s_basicVertices[i].position[1], s_basicVertices[i].position[2],0.0f);

			//Then we apply our rotation matrix on that point
			points = rotMat * points;
			
			//And at the end we just save those points back into our vertex list
			//So the new colors and positions of those points also get saved
			s_basicVertices[i].position[0] = points.getX(); 
			s_basicVertices[i].position[1] = points.getY();
			s_basicVertices[i].position[2] = points.getZ();

		}
	}

	//Just to reset the rotating angle
	//Generally to avoid any malfunction
	s_accumulatedRotatingAngle = 0;
	rotMat = Matrix4::identity();
	
}


float target(){
	//Too lazy to define pi on header
	float pi = 3.141592f;
	//We search the nearest pi
	float indice = searchNearestPi(pi);
	//we then find the angle, where the nearest pi is
	float tg = (pi/2) * indice;

	return tg;
}

float searchNearestPi(float pi){
	//Just a flag that will get substitude with the current smalles distance
	//between 2 angles.
	float minimum = 100;

	//Indice pointing to a point vector, which the currrent angle is near to
	//Also, this method returns the indice, where our nearest pi is
	int ind;
	//Tries 200 different combinations
	for(int i = -100; i < 100; i++){
		float distanceA = abs((pi/2) * i - s_accumulatedRotatingAngle);
		//Compare distances till we find the nearest angle of pi
		//We save that indice
		if(distanceA < minimum){
			minimum = distanceA;
			ind = i;
		}
	}
	return ind;
}

void controlsHandeling(){

	//We take the controller and save the input in a variable
	//either 0 or 1, 0 meaning no control input registered, 1 meaning there's a force applied
     SceCtrlData result;
     sceCtrlReadBufferPositive(0, &result, 1);

	 //To avoid using an accumulated angle that possibly can keep storing data from other resource
	 //(Meaning, not our desired current angle needed for the quaternion)
	 //We just set a new angle, so the cube stops, as soon as we stop moving the joystick
	  float s_newAccumulatedTurningAngleX = makeFloat(result.lx);
	  float s_newAccumulatedTurningAngleY = makeFloat(result.ly);

	 //Since the joystick is not set to 0 by default (Meaning, it's always gonna be moved by a bit) we check
	 //if there's a small force applied to our cube, it wont move by itself, only strong forces will affect our turning angle
	 //and move the cube
	  if(makeFloat(result.ly) > 0.5f || makeFloat(result.lx) > 0.5f || makeFloat(result.ly) < -0.5f || makeFloat(result.lx) < -0.5f){ 
		//We created an angle for the quaternion we our current accumulated turning angle
		Quat angle = Quat::Quat(s_newAccumulatedTurningAngleY, s_newAccumulatedTurningAngleX * -1, 0, 0);
		//Integration des Quaternions
		s_accumulatedQuaternion = s_accumulatedQuaternion + 0.06 * (factor * angle * s_accumulatedQuaternion);
		//Normalize the Quaternion
		s_accumulatedQuaternion = normalize(s_accumulatedQuaternion);
	  }
			
	 //Since the joystick is not set to 0 by default (Meaning, it's always gonna be moved by a bit) we check
	 //if there's a small force applied to our cube, it wont rotate the sides by itself, only strong forces will affect our turning angle
	 //and rotate the columns
	 if(makeFloat(result.ry) > 0.5f|| makeFloat(result.ry) < -0.5f){ 
		 smoothRotationFlag = false;
		 s_accumulatedRotatingAngle += makeFloat(result.ry) * 0.07f;
	}else{
		smoothRotationFlag = true;
	 }
	 //We store the nearest posible target angle to our current angle
	 float tg = target();

	 //We check if the animation for the rotation needs to be smoothed
	 if(smoothRotationFlag){
		 //If it does, we check if its near to the negative or near to the positive
		 //direction, and then we just progressively add a small factor
		 //till we reach the desired angle and lock it down there
		if(s_accumulatedRotatingAngle < tg - 0.03f){
			s_accumulatedRotatingAngle+=0.03f;
		}else if (s_accumulatedRotatingAngle > tg + 0.03f){
			s_accumulatedRotatingAngle-=0.03f;
		 
		}else{
			 //If we finished rotation our column we do the following

			 //1)
			 //Set rotation just saves the new rotated points and colors
			 setRotation();
			 
			 //2)
			 //Just to clear the flag back to 0 for smoothness
			 //And to avoid graphic glitches
			 cube.clearRotFlag(s_basicVertices);

			 //3)
			 //We set the smoothFlag back to false
			 //So it doesn't keep accumulating the rotation angle
			 smoothRotationFlag = false;

		}
  	}


	 //The Square button
	 if((result.buttons & SCE_CTRL_SQUARE) > 0){
		 //Flag to avoid any malfunction
		 if(!squarePressed){
			nextRotPoint = 0;
			squarePressed = true;
		 }
	 }else{
		squarePressed = false;
	 }

	 //The Triangle button
	 if((result.buttons & SCE_CTRL_TRIANGLE) > 0){
		 //Flag to avoid any malfunction
		 if(!trianglePressed){
			nextRotPoint = 1;
			trianglePressed = true;
		 }
	 }else{
		trianglePressed = false;
	 }

	 //The Circle button
	 if((result.buttons & SCE_CTRL_CIRCLE) > 0){
		 //Flag to avoid any malfunction
		 if(!circlePressed){
			nextRotPoint = 2;
			circlePressed = true;
		 }
	 }else{
		circlePressed = false;
	 }
	
	 //Here we handle the arrows 

	  if((result.buttons & SCE_CTRL_RIGHT) > 0){
		 //Flag to avoid any malfunction
		 if(!rightPressed){	
			 //Here we decide which direction we are taking
			 //Needed on rotating a give face of the cube
			 dir -= 2;
			 dir %= 3;
			 dir++;
			rightPressed = true;
		 }
	 }else{
		rightPressed = false;
	 }
	 if((result.buttons & SCE_CTRL_LEFT) > 0){
		 //Flag to avoid any malfunction
		 if(!leftPressed){
			 //Here we decide which direction we are taking
			 //Needed on rotating a give face of the cube
			 dir += 2;
			 dir %= 3;
			 dir--;
			leftPressed = true;
		 }
	 }else{
		leftPressed = false;
	 }

	
			 
}

//Report for the touch input
SceTouchReport globalTouchBack;
SceTouchReport globalTouchFront;

void touchHandeling(){

	//Back touch panel input
	SceTouchPanelInfo panelInfoBack; 
	sceTouchGetPanelInfo(SCE_TOUCH_PORT_BACK, &panelInfoBack);

	SceTouchData touchDataBack;

	sceTouchRead(SCE_TOUCH_PORT_BACK, &touchDataBack, 1);

	SceTouchReport touchReportBack = touchDataBack.report[0];

	//If there's a report of touch
	if(touchDataBack.reportNum > 0){
		//If the flag of back touch is still not activated
		if(!touchFlagBack){
			//We save that first point on a global report
			globalTouchBack.x = touchReportBack.x;
			globalTouchBack.y = touchReportBack.y;
			//And we set the flag to true to avoid malfunctioning
			touchFlagBack = true;
		}else{
			//When we are done swipeing we create a vector with the first touch and the last point
			//On we ended the swipe
			Vector2 origin = Vector2::Vector2(globalTouchBack.x, globalTouchBack.y);
			Vector2 dest = Vector2::Vector2(touchReportBack.x, touchReportBack.y);
			//Find the distance (difference) between those 2 points
			Vector2 distanceVector = origin - dest;
			//We caculate the length of this new vector
			float len = length(distanceVector);

			//We store the angle of our distance vector
			//we create a new angle to avoid malfunction
			float s_newAccumulated2TurningAngleX = makeFloat(distanceVector.getX());
			float s_newAccumulated2TurningAngleY = makeFloat(distanceVector.getY());
			if(len > 50 || len < -50){
				
				//We get the current angle with our accumulated angle for the quaternion
				Quat angle = Quat::Quat( abs(s_newAccumulated2TurningAngleY),  abs(s_newAccumulated2TurningAngleX) * -1, 0, 0);
				//Integration des Quaternions
				s_accumulatedQuaternion = s_accumulatedQuaternion + 0.06 * (factor * -angle * s_accumulatedQuaternion);
				//Normalize the Quaternion
				s_accumulatedQuaternion = normalize(s_accumulatedQuaternion);
				
				//We save that small step on our accumulated angle
				//s_accumulatedTurningAngleX = abs(s_newAccumulated2TurningAngleX) + makeFloat(distanceVector.getX())*0.01f;	
				//s_accumulatedTurningAngleY = abs(s_newAccumulated2TurningAngleY) + makeFloat(distanceVector.getY())*0.01f;	
			}
			

		}
	}else{
		touchFlagBack = false;
		s_accumulatedTurningAngleX = 0;
		s_accumulatedTurningAngleY = 0;
		
	}

	//Front touch panel input
	SceTouchPanelInfo panelInfoFront; 
	sceTouchGetPanelInfo(SCE_TOUCH_PORT_FRONT, &panelInfoFront);

	SceTouchData touchDataFront;

	sceTouchRead(SCE_TOUCH_PORT_FRONT, &touchDataFront, 1);

	SceTouchReport touchReportFront = touchDataFront.report[0];

	//Check if there's a finger in our current face
	if(touchDataFront.reportNum > 0){
		//Check if the touch hasn't been activated
		if(!touchFlagFront){
			//Save that point
			globalTouchFront.x = touchReportFront.x;
			globalTouchFront.y = touchReportFront.y;
			//Flag to avoid malfunction
			touchFlagFront = true;
		}else{
			//Save the origin points on a vector
			Vector2 origin2 = Vector2::Vector2(globalTouchFront.x, globalTouchFront.y);
			//Save the destination points on a vector
			Vector2 dest2 = Vector2::Vector2(touchReportFront.x, touchReportFront.y);
			//We calculate the distance vector between both point
			Vector2 distanceVector2 = origin2 - dest2;
			//Find the length
			float len2 = length(distanceVector2);
			//We check if the applied swipe overcomes a small distance before it can work
			if(len2 > 50){
				//Reset smooth flag
				smoothRotationFlag = false;
				//We save that small step on our accumulated angle
				s_accumulatedRotatingAngle += distanceVector2.getY() * 0.0005f;	
			}
			

		}
	}else{
		//Reset flag state
		touchFlagFront = false;
		
	}
}

/* Initialize libgxm */
int initGxm( void )
{
/* ---------------------------------------------------------------------
	2. Initialize libgxm

	First we must initialize the libgxm library by calling sceGxmInitialize.
	The single argument to this function is the size of the parameter buffer to
	allocate for the GPU.  We will use the default 16MiB here.

	Once initialized, we need to create a rendering context to allow to us
	to render scenes on the GPU.  We use the default initialization
	parameters here to set the sizes of the various context ring buffers.

	Finally we create a render target to describe the geometry of the back
	buffers we will render to.  This object is used purely to schedule
	rendering jobs for the given dimensions, the color surface and
	depth/stencil surface must be allocated separately.
	--------------------------------------------------------------------- */

	int returnCode = SCE_OK;

	/* set up parameters */
	SceGxmInitializeParams initializeParams;
	memset( &initializeParams, 0, sizeof(SceGxmInitializeParams) );
	initializeParams.flags = 0;
	initializeParams.displayQueueMaxPendingCount = DISPLAY_MAX_PENDING_SWAPS;
	initializeParams.displayQueueCallback = displayCallback;
	initializeParams.displayQueueCallbackDataSize = sizeof(DisplayData);
	initializeParams.parameterBufferSize = SCE_GXM_DEFAULT_PARAMETER_BUFFER_SIZE;

	/* start libgxm */
	returnCode = sceGxmInitialize( &initializeParams );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

	/* allocate ring buffer memory using default sizes */
	void *vdmRingBuffer = graphicsAlloc( SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, SCE_GXM_DEFAULT_VDM_RING_BUFFER_SIZE, 4, SCE_GXM_MEMORY_ATTRIB_READ, &s_vdmRingBufferUId );

	void *vertexRingBuffer = graphicsAlloc( SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, SCE_GXM_DEFAULT_VERTEX_RING_BUFFER_SIZE, 4, SCE_GXM_MEMORY_ATTRIB_READ, &s_vertexRingBufferUId );

	void *fragmentRingBuffer = graphicsAlloc( SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, SCE_GXM_DEFAULT_FRAGMENT_RING_BUFFER_SIZE, 4, SCE_GXM_MEMORY_ATTRIB_READ, &s_fragmentRingBufferUId );

	uint32_t fragmentUsseRingBufferOffset;
	void *fragmentUsseRingBuffer = fragmentUsseAlloc( SCE_GXM_DEFAULT_FRAGMENT_USSE_RING_BUFFER_SIZE, &s_fragmentUsseRingBufferUId, &fragmentUsseRingBufferOffset );

	/* create a rendering context */
	memset( &s_contextParams, 0, sizeof(SceGxmContextParams) );
	s_contextParams.hostMem = malloc( SCE_GXM_MINIMUM_CONTEXT_HOST_MEM_SIZE );
	s_contextParams.hostMemSize = SCE_GXM_MINIMUM_CONTEXT_HOST_MEM_SIZE;
	s_contextParams.vdmRingBufferMem = vdmRingBuffer;
	s_contextParams.vdmRingBufferMemSize = SCE_GXM_DEFAULT_VDM_RING_BUFFER_SIZE;
	s_contextParams.vertexRingBufferMem = vertexRingBuffer;
	s_contextParams.vertexRingBufferMemSize = SCE_GXM_DEFAULT_VERTEX_RING_BUFFER_SIZE;
	s_contextParams.fragmentRingBufferMem = fragmentRingBuffer;
	s_contextParams.fragmentRingBufferMemSize = SCE_GXM_DEFAULT_FRAGMENT_RING_BUFFER_SIZE;
	s_contextParams.fragmentUsseRingBufferMem = fragmentUsseRingBuffer;
	s_contextParams.fragmentUsseRingBufferMemSize = SCE_GXM_DEFAULT_FRAGMENT_USSE_RING_BUFFER_SIZE;
	s_contextParams.fragmentUsseRingBufferOffset = fragmentUsseRingBufferOffset;
	returnCode = sceGxmCreateContext( &s_contextParams, &s_context );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

	/* set buffer sizes for this sample */
	const uint32_t patcherBufferSize = 64*1024;
	const uint32_t patcherVertexUsseSize = 64*1024;
	const uint32_t patcherFragmentUsseSize = 64*1024;

	/* allocate memory for buffers and USSE code */
	void *patcherBuffer = graphicsAlloc( SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, patcherBufferSize, 4, SCE_GXM_MEMORY_ATTRIB_WRITE|SCE_GXM_MEMORY_ATTRIB_WRITE, &s_patcherBufferUId );

	uint32_t patcherVertexUsseOffset;
	void *patcherVertexUsse = vertexUsseAlloc( patcherVertexUsseSize, &s_patcherVertexUsseUId, &patcherVertexUsseOffset );

	uint32_t patcherFragmentUsseOffset;
	void *patcherFragmentUsse = fragmentUsseAlloc( patcherFragmentUsseSize, &s_patcherFragmentUsseUId, &patcherFragmentUsseOffset );

	/* create a shader patcher */
	SceGxmShaderPatcherParams patcherParams;
	memset( &patcherParams, 0, sizeof(SceGxmShaderPatcherParams) );
	patcherParams.userData = NULL;
	patcherParams.hostAllocCallback = &patcherHostAlloc;
	patcherParams.hostFreeCallback = &patcherHostFree;
	patcherParams.bufferAllocCallback = NULL;
	patcherParams.bufferFreeCallback = NULL;
	patcherParams.bufferMem = patcherBuffer;
	patcherParams.bufferMemSize = patcherBufferSize;
	patcherParams.vertexUsseAllocCallback = NULL;
	patcherParams.vertexUsseFreeCallback = NULL;
	patcherParams.vertexUsseMem = patcherVertexUsse;
	patcherParams.vertexUsseMemSize = patcherVertexUsseSize;
	patcherParams.vertexUsseOffset = patcherVertexUsseOffset;
	patcherParams.fragmentUsseAllocCallback = NULL;
	patcherParams.fragmentUsseFreeCallback = NULL;
	patcherParams.fragmentUsseMem = patcherFragmentUsse;
	patcherParams.fragmentUsseMemSize = patcherFragmentUsseSize;
	patcherParams.fragmentUsseOffset = patcherFragmentUsseOffset;
	returnCode = sceGxmShaderPatcherCreate( &patcherParams, &s_shaderPatcher );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

	/* create a render target */
	memset( &s_renderTargetParams, 0, sizeof(SceGxmRenderTargetParams) );
	s_renderTargetParams.flags = 0;
	s_renderTargetParams.width = DISPLAY_WIDTH;
	s_renderTargetParams.height = DISPLAY_HEIGHT;
	s_renderTargetParams.scenesPerFrame = 1;
	s_renderTargetParams.multisampleMode = SCE_GXM_MULTISAMPLE_NONE;
	s_renderTargetParams.multisampleLocations	= 0;
	s_renderTargetParams.driverMemBlock = SCE_UID_INVALID_UID;

	returnCode = sceGxmCreateRenderTarget( &s_renderTargetParams, &s_renderTarget );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );


/* ---------------------------------------------------------------------
	3. Allocate display buffers, set up the display queue

	We will allocate our back buffers in CDRAM, and create a color
	surface for each of them.

	To allow display operations done by the CPU to be synchronized with
	rendering done by the GPU, we also create a SceGxmSyncObject for each
	display buffer.  This sync object will be used with each scene that
	renders to that buffer and when queueing display flips that involve
	that buffer (either flipping from or to).

	Finally we create a display queue object that points to our callback
	function.
	--------------------------------------------------------------------- */

	/* allocate memory and sync objects for display buffers */
	for ( unsigned int i = 0 ; i < DISPLAY_BUFFER_COUNT ; ++i )
	{
		/* allocate memory with large size to ensure physical contiguity */
		s_displayBufferData[i] = graphicsAlloc( SCE_KERNEL_MEMBLOCK_TYPE_USER_CDRAM_RWDATA, ALIGN(4*DISPLAY_STRIDE_IN_PIXELS*DISPLAY_HEIGHT, 1*1024*1024), SCE_GXM_COLOR_SURFACE_ALIGNMENT, SCE_GXM_MEMORY_ATTRIB_READ|SCE_GXM_MEMORY_ATTRIB_WRITE, &s_displayBufferUId[i] );
		SCE_DBG_ALWAYS_ASSERT( s_displayBufferData[i] );

		/* memset the buffer to debug color */
		for ( unsigned int y = 0 ; y < DISPLAY_HEIGHT ; ++y )
		{
			unsigned int *row = (unsigned int *)s_displayBufferData[i] + y*DISPLAY_STRIDE_IN_PIXELS;

			for ( unsigned int x = 0 ; x < DISPLAY_WIDTH ; ++x )
			{
				row[x] = 0x0;
			}
		}

		/* initialize a color surface for this display buffer */
		returnCode = sceGxmColorSurfaceInit( &s_displaySurface[i], DISPLAY_COLOR_FORMAT, SCE_GXM_COLOR_SURFACE_LINEAR, SCE_GXM_COLOR_SURFACE_SCALE_NONE,
											 SCE_GXM_OUTPUT_REGISTER_SIZE_32BIT, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_STRIDE_IN_PIXELS, s_displayBufferData[i] );
		SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

		/* create a sync object that we will associate with this buffer */
		returnCode = sceGxmSyncObjectCreate( &s_displayBufferSync[i] );
		SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );
	}

	/* compute the memory footprint of the depth buffer */
	const uint32_t alignedWidth = ALIGN( DISPLAY_WIDTH, SCE_GXM_TILE_SIZEX );
	const uint32_t alignedHeight = ALIGN( DISPLAY_HEIGHT, SCE_GXM_TILE_SIZEY );
	uint32_t sampleCount = alignedWidth*alignedHeight;
	uint32_t depthStrideInSamples = alignedWidth;

	/* allocate it */
	void *depthBufferData = graphicsAlloc( SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, 4*sampleCount, SCE_GXM_DEPTHSTENCIL_SURFACE_ALIGNMENT, SCE_GXM_MEMORY_ATTRIB_READ|SCE_GXM_MEMORY_ATTRIB_WRITE, &s_depthBufferUId );

	/* create the SceGxmDepthStencilSurface structure */
	returnCode = sceGxmDepthStencilSurfaceInit( &s_depthSurface, SCE_GXM_DEPTH_STENCIL_FORMAT_S8D24, SCE_GXM_DEPTH_STENCIL_SURFACE_TILED, depthStrideInSamples, depthBufferData, NULL );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

	return returnCode;
}





/* Create libgxm scenes */
void createGxmData( void )
{
/* ---------------------------------------------------------------------
	4. Create a shader patcher and register programs

	A shader patcher object is required to produce vertex and fragment
	programs from the shader compiler output.  First we create a shader
	patcher instance, using callback functions to allow it to allocate
	and free host memory for internal state.

	In order to create vertex and fragment programs for a particular
	shader, the compiler output must first be registered to obtain an ID
	for that shader.  Within a single ID, vertex and fragment programs
	are reference counted and could be shared if created with identical
	parameters.  To maximise this sharing, programs should only be
	registered with the shader patcher once if possible, so we will do
	this now.
	--------------------------------------------------------------------- */

	/* register programs with the patcher */
	int returnCode = sceGxmShaderPatcherRegisterProgram( s_shaderPatcher, &binaryClearVGxpStart, &s_clearVertexProgramId );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );
	returnCode = sceGxmShaderPatcherRegisterProgram( s_shaderPatcher, &binaryClearFGxpStart, &s_clearFragmentProgramId );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );


    returnCode = sceGxmShaderPatcherRegisterProgram( s_shaderPatcher, &binaryBasicVGxpStart, &s_basicVertexProgramId );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );
	returnCode = sceGxmShaderPatcherRegisterProgram( s_shaderPatcher, &binaryBasicFGxpStart, &s_basicFragmentProgramId );
    SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );


/* ---------------------------------------------------------------------
	5. Create the programs and data for the clear

	On SGX hardware, vertex programs must perform the unpack operations
	on vertex data, so we must define our vertex formats in order to
	create the vertex program.  Similarly, fragment programs must be
	specialized based on how they output their pixels and MSAA mode
	(and texture format on ES1).

	We define the clear geometry vertex format here and create the vertex
	and fragment program.

	The clear vertex and index data is static, we allocate and write the
	data here.
	--------------------------------------------------------------------- */

	/* get attributes by name to create vertex format bindings */
	const SceGxmProgram *clearProgram = sceGxmShaderPatcherGetProgramFromId( s_clearVertexProgramId );
	SCE_DBG_ALWAYS_ASSERT( clearProgram );
	const SceGxmProgramParameter *paramClearPositionAttribute = sceGxmProgramFindParameterByName( clearProgram, "aPosition" );
	SCE_DBG_ALWAYS_ASSERT( paramClearPositionAttribute && ( sceGxmProgramParameterGetCategory(paramClearPositionAttribute) == SCE_GXM_PARAMETER_CATEGORY_ATTRIBUTE ) );

	/* create clear vertex format */
	SceGxmVertexAttribute clearVertexAttributes[1];
	SceGxmVertexStream clearVertexStreams[1];
	clearVertexAttributes[0].streamIndex = 0;
	clearVertexAttributes[0].offset = 0;
	clearVertexAttributes[0].format = SCE_GXM_ATTRIBUTE_FORMAT_F32;
	clearVertexAttributes[0].componentCount = 2;
	clearVertexAttributes[0].regIndex = sceGxmProgramParameterGetResourceIndex( paramClearPositionAttribute );
	clearVertexStreams[0].stride = sizeof(ClearVertex);
	clearVertexStreams[0].indexSource = SCE_GXM_INDEX_SOURCE_INDEX_16BIT;

	/* create clear programs */
	returnCode = sceGxmShaderPatcherCreateVertexProgram( s_shaderPatcher, s_clearVertexProgramId, clearVertexAttributes, 1, clearVertexStreams, 1, &s_clearVertexProgram );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

	returnCode = sceGxmShaderPatcherCreateFragmentProgram( s_shaderPatcher, s_clearFragmentProgramId,
														   SCE_GXM_OUTPUT_REGISTER_FORMAT_UCHAR4, SCE_GXM_MULTISAMPLE_NONE, NULL,
														   sceGxmShaderPatcherGetProgramFromId(s_clearVertexProgramId), &s_clearFragmentProgram );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

	/* create the clear triangle vertex/index data */
	s_clearVertices = (ClearVertex *)graphicsAlloc( SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, 3*sizeof(ClearVertex), 4, SCE_GXM_MEMORY_ATTRIB_READ, &s_clearVerticesUId );
	s_clearIndices = (uint16_t *)graphicsAlloc( SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, 3*sizeof(uint16_t), 2, SCE_GXM_MEMORY_ATTRIB_READ, &s_clearIndicesUId );

	s_clearVertices[0].x = -1.0f;
	s_clearVertices[0].y = -1.0f;
	s_clearVertices[1].x =  3.0f;
	s_clearVertices[1].y = -1.0f;
	s_clearVertices[2].x = -1.0f;
	s_clearVertices[2].y =  3.0f;

	s_clearIndices[0] = 0;
	s_clearIndices[1] = 1;
	s_clearIndices[2] = 2;

    // !! All related to triangle.

    /* get attributes by name to create vertex format bindings */
	/* first retrieve the underlying program to extract binding information */
	const SceGxmProgram *basicProgram = sceGxmShaderPatcherGetProgramFromId( s_basicVertexProgramId );
	SCE_DBG_ALWAYS_ASSERT( basicProgram );
	const SceGxmProgramParameter *paramBasicPositionAttribute = sceGxmProgramFindParameterByName( basicProgram, "aPosition" );
	SCE_DBG_ALWAYS_ASSERT( paramBasicPositionAttribute && ( sceGxmProgramParameterGetCategory(paramBasicPositionAttribute) == SCE_GXM_PARAMETER_CATEGORY_ATTRIBUTE ) );
	const SceGxmProgramParameter *paramBasicColorAttribute = sceGxmProgramFindParameterByName( basicProgram, "aColor" );
	SCE_DBG_ALWAYS_ASSERT( paramBasicColorAttribute && ( sceGxmProgramParameterGetCategory(paramBasicColorAttribute) == SCE_GXM_PARAMETER_CATEGORY_ATTRIBUTE ) );
	const SceGxmProgramParameter *paramBasicFlagAttribute = sceGxmProgramFindParameterByName( basicProgram, "aRot" );
	SCE_DBG_ALWAYS_ASSERT( paramBasicFlagAttribute && ( sceGxmProgramParameterGetCategory(paramBasicFlagAttribute) == SCE_GXM_PARAMETER_CATEGORY_ATTRIBUTE ) );

	/* create shaded triangle vertex format */
	SceGxmVertexAttribute basicVertexAttributes[3];
	SceGxmVertexStream basicVertexStreams[1];
	basicVertexAttributes[0].streamIndex = 0;
	basicVertexAttributes[0].offset = 0;
	basicVertexAttributes[0].format = SCE_GXM_ATTRIBUTE_FORMAT_F32;
	basicVertexAttributes[0].componentCount = 3;
	basicVertexAttributes[0].regIndex = sceGxmProgramParameterGetResourceIndex( paramBasicPositionAttribute );
	basicVertexAttributes[1].streamIndex = 0;
	basicVertexAttributes[1].offset = 12;
	basicVertexAttributes[1].format = SCE_GXM_ATTRIBUTE_FORMAT_U8N; // Mapping relation clarified.
	basicVertexAttributes[1].componentCount = 4;
	basicVertexAttributes[1].regIndex = sceGxmProgramParameterGetResourceIndex( paramBasicColorAttribute );
	basicVertexAttributes[2].streamIndex = 0;
	basicVertexAttributes[2].offset = 16;
	basicVertexAttributes[2].format = SCE_GXM_ATTRIBUTE_FORMAT_U8;
	basicVertexAttributes[2].componentCount = 1;
	basicVertexAttributes[2].regIndex = sceGxmProgramParameterGetResourceIndex( paramBasicFlagAttribute );
	basicVertexStreams[0].stride = sizeof(Vertex);
	basicVertexStreams[0].indexSource = SCE_GXM_INDEX_SOURCE_INDEX_16BIT;

	/* create shaded triangle shaders */
	returnCode = sceGxmShaderPatcherCreateVertexProgram( s_shaderPatcher, s_basicVertexProgramId, basicVertexAttributes, 3,
														 basicVertexStreams, 1, &s_basicVertexProgram );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

	returnCode = sceGxmShaderPatcherCreateFragmentProgram( s_shaderPatcher, s_basicFragmentProgramId,
														   SCE_GXM_OUTPUT_REGISTER_FORMAT_UCHAR4, SCE_GXM_MULTISAMPLE_NONE, NULL,
														   sceGxmShaderPatcherGetProgramFromId(s_basicVertexProgramId), &s_basicFragmentProgram );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

	/* find vertex uniforms by name and cache parameter information */
	s_wvpParam = sceGxmProgramFindParameterByName( basicProgram, "wvp" );
	SCE_DBG_ALWAYS_ASSERT( s_wvpParam && ( sceGxmProgramParameterGetCategory( s_wvpParam ) == SCE_GXM_PARAMETER_CATEGORY_UNIFORM ) );

	s_rotParam = sceGxmProgramFindParameterByName( basicProgram, "rot" );
	SCE_DBG_ALWAYS_ASSERT( s_rotParam && ( sceGxmProgramParameterGetCategory( s_rotParam ) == SCE_GXM_PARAMETER_CATEGORY_UNIFORM ) );
	
	/* create shaded triangle vertex/index data */
	s_basicVertices = (Vertex *)graphicsAlloc( SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, 9 * 4 * 6 *sizeof(Vertex), 4, SCE_GXM_MEMORY_ATTRIB_READ, &s_basicVerticesUId );
	s_basicIndices = (uint16_t *)graphicsAlloc( SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, 9 * 6 * 6  *sizeof(uint16_t), 2, SCE_GXM_MEMORY_ATTRIB_READ, &s_basicIndiceUId );

	
    // The vertices of our cube are created here.
    cube.makeCube(s_basicVertices);
	// The indices of the triangles send to the vertex shader are set here.
	cube.makeIndices(s_basicIndices);


}




/* Main render function */
void render( void )
{
	/* render libgxm scenes */
	renderGxm();
}

/* render gxm scenes */
void renderGxm( void )
{
/* -----------------------------------------------------------------
	8. Rendering step

	This sample renders a single scene containing the clear triangle.
	Before any drawing can take place, a scene must be started.
	We render to the back buffer, so it is also important to use a
	sync object to ensure that these rendering operations are
	synchronized with display operations.

	The clear triangle shaders do not declare any uniform variables,
	so this may be rendered immediately after setting the vertex and
	fragment program.

	Once clear triangle have been drawn the scene can be ended, which
	submits it for rendering on the GPU.
	----------------------------------------------------------------- */

	/* start rendering to the render target */
	sceGxmBeginScene( s_context, 0, s_renderTarget, NULL, NULL, s_displayBufferSync[s_displayBackBufferIndex], &s_displaySurface[s_displayBackBufferIndex], &s_depthSurface );

	/* set clear shaders */
	sceGxmSetVertexProgram( s_context, s_clearVertexProgram );
	sceGxmSetFragmentProgram( s_context, s_clearFragmentProgram );

	/* draw ther clear triangle */
	sceGxmSetVertexStream( s_context, 0, s_clearVertices );
	sceGxmDraw( s_context, SCE_GXM_PRIMITIVE_TRIANGLES, SCE_GXM_INDEX_FORMAT_U16, s_clearIndices, 3 );


    // !! Speciality for rendering a triangle.

    /* render the  triangle */
	sceGxmSetVertexProgram( s_context, s_basicVertexProgram );
	sceGxmSetFragmentProgram( s_context, s_basicFragmentProgram );

	/* set the vertex program constants */
	void *vertexDefaultBuffer;
	sceGxmReserveVertexDefaultUniformBuffer( s_context, &vertexDefaultBuffer );
	sceGxmSetUniformDataF( vertexDefaultBuffer, s_wvpParam, 0, 16, (float*)&s_finalTransformation );
	sceGxmSetUniformDataF( vertexDefaultBuffer, s_rotParam, 0, 16, (float*)&s_finalColumnRotation);

	/* draw the spinning triangle */
	sceGxmSetVertexStream( s_context, 0, s_basicVertices );
	sceGxmDraw( s_context, SCE_GXM_PRIMITIVE_TRIANGLES, SCE_GXM_INDEX_FORMAT_U16, s_basicIndices, 6 * 6 * 9 );

	/* stop rendering to the render target */
	sceGxmEndScene( s_context, NULL, NULL );
}



/* queue a display swap and cycle our buffers */
void cycleDisplayBuffers( void )
{
/* -----------------------------------------------------------------
	9-a. Flip operation

	Now we have finished submitting rendering work for this frame it
	is time to submit a flip operation.  As part of specifying this
	flip operation we must provide the sync objects for both the old
	buffer and the new buffer.  This is to allow synchronization both
	ways: to not flip until rendering is complete, but also to ensure
	that future rendering to these buffers does not start until the
	flip operation is complete.

	Once we have queued our flip, we manually cycle through our back
	buffers before starting the next frame.
	----------------------------------------------------------------- */

	/* PA heartbeat to notify end of frame */
	sceGxmPadHeartbeat( &s_displaySurface[s_displayBackBufferIndex], s_displayBufferSync[s_displayBackBufferIndex] );

	/* queue the display swap for this frame */
	DisplayData displayData;
	displayData.address = s_displayBufferData[s_displayBackBufferIndex];

	/* front buffer is OLD buffer, back buffer is NEW buffer */
	sceGxmDisplayQueueAddEntry( s_displayBufferSync[s_displayFrontBufferIndex], s_displayBufferSync[s_displayBackBufferIndex], &displayData );

	/* update buffer indices */
	s_displayFrontBufferIndex = s_displayBackBufferIndex;
	s_displayBackBufferIndex = (s_displayBackBufferIndex + 1) % DISPLAY_BUFFER_COUNT;
}


/* Destroy Gxm Data */
void destroyGxmData( void )
{
/* ---------------------------------------------------------------------
	11. Destroy the programs and data for the clear and spinning triangle

	Once the GPU is finished, we release all our programs.
	--------------------------------------------------------------------- */

	/* clean up allocations */
	sceGxmShaderPatcherReleaseFragmentProgram( s_shaderPatcher, s_clearFragmentProgram );
	sceGxmShaderPatcherReleaseVertexProgram( s_shaderPatcher, s_clearVertexProgram );
	graphicsFree( s_clearIndicesUId );
	graphicsFree( s_clearVerticesUId );

	/* wait until display queue is finished before deallocating display buffers */
	sceGxmDisplayQueueFinish();

	/* unregister programs and destroy shader patcher */
	sceGxmShaderPatcherUnregisterProgram( s_shaderPatcher, s_clearFragmentProgramId );
	sceGxmShaderPatcherUnregisterProgram( s_shaderPatcher, s_clearVertexProgramId );
	sceGxmShaderPatcherDestroy( s_shaderPatcher );
	fragmentUsseFree( s_patcherFragmentUsseUId );
	vertexUsseFree( s_patcherVertexUsseUId );
	graphicsFree( s_patcherBufferUId );
}



/* ShutDown libgxm */
int shutdownGxm( void )
{
/* ---------------------------------------------------------------------
	12. Finalize libgxm

	Once the GPU is finished, we deallocate all our memory,
	destroy all object and finally terminate libgxm.
	--------------------------------------------------------------------- */

	int returnCode = SCE_OK;

	graphicsFree( s_depthBufferUId );

	for ( unsigned int i = 0 ; i < DISPLAY_BUFFER_COUNT; ++i )
	{
		memset( s_displayBufferData[i], 0, DISPLAY_HEIGHT*DISPLAY_STRIDE_IN_PIXELS*4 );
		graphicsFree( s_displayBufferUId[i] );
		sceGxmSyncObjectDestroy( s_displayBufferSync[i] );
	}

	/* destroy the render target */
	sceGxmDestroyRenderTarget( s_renderTarget );

	/* destroy the context */
	sceGxmDestroyContext( s_context );

	fragmentUsseFree( s_fragmentUsseRingBufferUId );
	graphicsFree( s_fragmentRingBufferUId );
	graphicsFree( s_vertexRingBufferUId );
	graphicsFree( s_vdmRingBufferUId );
	free( s_contextParams.hostMem );

	/* terminate libgxm */
	sceGxmTerminate();

	return returnCode;
}


/* Host alloc */
static void *patcherHostAlloc( void *userData, unsigned int size )
{
	(void)( userData );

	return malloc( size );
}

/* Host free */
static void patcherHostFree( void *userData, void *mem )
{
	(void)( userData );

	free( mem );
}

/* Display callback */
void displayCallback( const void *callbackData )
{
/* -----------------------------------------------------------------
	10-b. Flip operation

	The callback function will be called from an internal thread once
	queued GPU operations involving the sync objects is complete.
	Assuming we have not reached our maximum number of queued frames,
	this function returns immediately.
	----------------------------------------------------------------- */

	SceDisplayFrameBuf framebuf;

	/* cast the parameters back */
	const DisplayData *displayData = (const DisplayData *)callbackData;


    // Render debug text.
    /* set framebuffer info */
	SceDbgFontFrameBufInfo info;
	memset( &info, 0, sizeof(SceDbgFontFrameBufInfo) );
	info.frameBufAddr = (SceUChar8 *)displayData->address;
	info.frameBufPitch = DISPLAY_STRIDE_IN_PIXELS;
	info.frameBufWidth = DISPLAY_WIDTH;
	info.frameBufHeight = DISPLAY_HEIGHT;
	info.frameBufPixelformat = DBGFONT_PIXEL_FORMAT;

	/* flush font buffer */
	int returnCode = sceDbgFontFlush( &info );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );
	

	/* wwap to the new buffer on the next VSYNC */
	memset(&framebuf, 0x00, sizeof(SceDisplayFrameBuf));
	framebuf.size        = sizeof(SceDisplayFrameBuf);
	framebuf.base        = displayData->address;
	framebuf.pitch       = DISPLAY_STRIDE_IN_PIXELS;
	framebuf.pixelformat = DISPLAY_PIXEL_FORMAT;
	framebuf.width       = DISPLAY_WIDTH;
	framebuf.height      = DISPLAY_HEIGHT;
	returnCode = sceDisplaySetFrameBuf( &framebuf, SCE_DISPLAY_UPDATETIMING_NEXTVSYNC );
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );

	/* block this callback until the swap has occurred and the old buffer is no longer displayed */
	returnCode = sceDisplayWaitVblankStart();
	SCE_DBG_ALWAYS_ASSERT( returnCode == SCE_OK );
}

/* Alloc used by libgxm */
static void *graphicsAlloc( SceKernelMemBlockType type, uint32_t size, uint32_t alignment, uint32_t attribs, SceUID *uid )
{
/*	Since we are using sceKernelAllocMemBlock directly, we cannot directly
	use the alignment parameter.  Instead, we must allocate the size to the
	minimum for this memblock type, and just SCE_DBG_ALWAYS_ASSERT that this will cover
	our desired alignment.

	Developers using their own heaps should be able to use the alignment
	parameter directly for more minimal padding.
*/

	if( type == SCE_KERNEL_MEMBLOCK_TYPE_USER_CDRAM_RWDATA )
	{
		/* CDRAM memblocks must be 256KiB aligned */
		SCE_DBG_ALWAYS_ASSERT( alignment <= 256*1024 );
		size = ALIGN( size, 256*1024 );
	}
	else
	{
		/* LPDDR memblocks must be 4KiB aligned */
		SCE_DBG_ALWAYS_ASSERT( alignment <= 4*1024 );
		size = ALIGN( size, 4*1024 );
	}

	/* allocate some memory */
	*uid = sceKernelAllocMemBlock( "simple", type, size, NULL );
	SCE_DBG_ALWAYS_ASSERT( *uid >= SCE_OK );

	/* grab the base address */
	void *mem = NULL;
	int err = sceKernelGetMemBlockBase( *uid, &mem );
	SCE_DBG_ALWAYS_ASSERT( err == SCE_OK );

	/* map for the GPU */
	err = sceGxmMapMemory( mem, size, attribs );
	SCE_DBG_ALWAYS_ASSERT( err == SCE_OK );

	/* done */
	return mem;
}

/* Free used by libgxm */
static void graphicsFree( SceUID uid )
{
	/* grab the base address */
	void *mem = NULL;
	int err = sceKernelGetMemBlockBase(uid, &mem);
	SCE_DBG_ALWAYS_ASSERT(err == SCE_OK);

	// unmap memory
	err = sceGxmUnmapMemory(mem);
	SCE_DBG_ALWAYS_ASSERT(err == SCE_OK);

	// free the memory block
	err = sceKernelFreeMemBlock(uid);
	SCE_DBG_ALWAYS_ASSERT(err == SCE_OK);
}

/* vertex alloc used by libgxm */
static void *vertexUsseAlloc( uint32_t size, SceUID *uid, uint32_t *usseOffset )
{
	/* align to memblock alignment for LPDDR */
	size = ALIGN( size, 4096 );

	/* allocate some memory */
	*uid = sceKernelAllocMemBlock( "simple", SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, size, NULL );
	SCE_DBG_ALWAYS_ASSERT( *uid >= SCE_OK );

	/* grab the base address */
	void *mem = NULL;
	int err = sceKernelGetMemBlockBase( *uid, &mem );
	SCE_DBG_ALWAYS_ASSERT( err == SCE_OK );

	/* map as vertex USSE code for the GPU */
	err = sceGxmMapVertexUsseMemory( mem, size, usseOffset );
	SCE_DBG_ALWAYS_ASSERT( err == SCE_OK );

	return mem;
}

/* vertex free used by libgxm */
static void vertexUsseFree( SceUID uid )
{
	/* grab the base address */
	void *mem = NULL;
	int err = sceKernelGetMemBlockBase( uid, &mem );
	SCE_DBG_ALWAYS_ASSERT(err == SCE_OK);

	/* unmap memory */
	err = sceGxmUnmapVertexUsseMemory( mem );
	SCE_DBG_ALWAYS_ASSERT(err == SCE_OK);

	/* free the memory block */
	err = sceKernelFreeMemBlock( uid );
	SCE_DBG_ALWAYS_ASSERT( err == SCE_OK );
}

/* fragment alloc used by libgxm */
static void *fragmentUsseAlloc( uint32_t size, SceUID *uid, uint32_t *usseOffset )
{
	/* align to memblock alignment for LPDDR */
	size = ALIGN( size, 4096 );

	/* allocate some memory */
	*uid = sceKernelAllocMemBlock( "simple", SCE_KERNEL_MEMBLOCK_TYPE_USER_RWDATA_UNCACHE, size, NULL );
	SCE_DBG_ALWAYS_ASSERT( *uid >= SCE_OK );

	/* grab the base address */
	void *mem = NULL;
	int err = sceKernelGetMemBlockBase( *uid, &mem );
	SCE_DBG_ALWAYS_ASSERT( err == SCE_OK );

	/* map as fragment USSE code for the GPU */
	err = sceGxmMapFragmentUsseMemory( mem, size, usseOffset);
	SCE_DBG_ALWAYS_ASSERT(err == SCE_OK);

	// done
	return mem;
}

/* fragment free used by libgxm */
static void fragmentUsseFree( SceUID uid )
{
	/* grab the base address */
	void *mem = NULL;
	int err = sceKernelGetMemBlockBase( uid, &mem );
	SCE_DBG_ALWAYS_ASSERT( err == SCE_OK );

	/* unmap memory */
	err = sceGxmUnmapFragmentUsseMemory( mem );
	SCE_DBG_ALWAYS_ASSERT( err == SCE_OK );

	/* free the memory block */
	err = sceKernelFreeMemBlock( uid );
	SCE_DBG_ALWAYS_ASSERT( err == SCE_OK );
}

