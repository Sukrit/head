#include <math.h>
#include <string.h>
#include <ode/ode.h>
#include <algorithm>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"


#ifdef WIN32
#include <windows.h>
#endif

#include <ode/ode.h>
#include "config.h"
#include <stdlib.h>

#ifdef HAVE_APPLE_OPENGL_FRAMEWORK
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "WinGDI.h"
#endif

using namespace std;

#define PI 3.14159265
#define Kp 2500.0
#define Kd 20
#define Kp_force 10000
#define Kd_force 500
#define Kp_velocity 1500
#define Kd_velocity 50

#define TIME_STEP 0.001

// You can choose the window size
#define WINDOW_WIDTH 700
#define WINDOW_HEIGHT 500

// used to allocate space for robot info.
#define MAX_N_ROBOT_PARTS 100


// the robot
static dBodyID robot_bodies[MAX_N_ROBOT_PARTS];
static dGeomID robot_geoms[MAX_N_ROBOT_PARTS];
int n_robot_parts = 0;
static dJointID robot_joints[MAX_N_ROBOT_PARTS];
int n_robot_joints = 0;
static dReal joint_positions[MAX_N_ROBOT_PARTS];
static dReal joint_velocities[MAX_N_ROBOT_PARTS];
static dReal joint_torques[MAX_N_ROBOT_PARTS];
static dReal joint_positions_desired[MAX_N_ROBOT_PARTS];
static dReal k_pos[MAX_N_ROBOT_PARTS];
static dReal k_vel[MAX_N_ROBOT_PARTS];

#define FOOT_LENGTH 0.1
#define FOOT_WIDTH 0.08
#define FOOT_THICKNESS 0.04
#define FOOT_MASS 3.0

#define SHANK_RADIUS 0.04
#define SHANK_HEIGHT 0.15
#define SHANK_MASS 5
#define SHANK_Z SHANK_HEIGHT/2
#define THIGH_MASS 5

#define LEG_X 0.25
#define LEG_Y 0.14

#define LEG_Y2 LEG_Y
#define FOOT_Z FOOT_THICKNESS/2
#define THIGH_Z SHANK_HEIGHT*3/2 - FOOT_THICKNESS
#define THIGH_JOINT_HEIGHT 2*FOOT_THICKNESS
#define THIGH_JOINT_Z 2*SHANK_HEIGHT - FOOT_THICKNESS

#define TORSO_X LEG_X
#define TORSO_Y 0
#define TORSO_Z 2*SHANK_HEIGHT

#define NECK_RADIUS 0.05
#define NECK_HEIGHT 0.1
#define NECK_MASS 0.1
#define NECK_X 0.34
#define NECK_Y 0
#define NECK_Z TORSO_Z + 0.12

#define HEAD_RADIUS 0.07
#define HEAD_HEIGHT 0.12
#define HEAD_X 0.44
#define HEAD_Y 0 
#define HEAD_Z TORSO_Z + 0.13
#define HEAD_MASS 0.5

#define TAIL_X -LEG_X/2
#define TAIL_Y 0
#define TAIL_Z TORSO_Z
#define TAIL_LENGTH 0.07
#define TAIL_RADIUS 0.02
#define TAIL_MASS 0.01

#define BOX_LENGTH 0.1
#define BOX_WIDTH 2*LEG_Y-2*SHANK_RADIUS
#define BOX_THICKNESS SHANK_HEIGHT*1.2
#define TORSO_BOX_MASS 2


// select correct drawing functions (we are using double version)
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#define dsDrawTriangle dsDrawTriangleD
#define dsDrawLine dsDrawLineD
#endif

#define WALK_STRIDE_TIME 1

static dReal time = 0; // what time is it
static dReal walk_time = 0; // what time is it



dsFunctions fn;
static dWorldID world;
static dSpaceID space;
static dGeomID plane;
static dJointGroupID contact_joint_group;
static dJointGroupID robot_joint_group;
static dJointID club_joint;


// the current feedforward torque
dReal club_joint_ff = 0;
int walk_start_back = 0;
int walk_start_front = 0;

static dReal velocity = 1.0;
static dReal stride_phase = 0.005; 
static dReal front_left_phase = 0.0835;
static dReal front_right_phase = 0.9185;
static dReal back_left_phase = 0.9185;
static dReal back_right_phase = 0.0835;
static const dReal *Back_left_P1;
static const dReal *Back_right_P1;
static const dReal *Front_left_P1;
static const dReal *Front_right_P1;
static dVector3 back_left_P1;
static dVector3 back_right_P1;
static dVector3 front_left_P1;
static dVector3 front_right_P1;
static dVector3 back_left_P2;
static dVector3 back_right_P2;
static dVector3 front_left_P2;
static dVector3 front_right_P2;
static dVector3 back_left_P;
static dVector3 back_right_P;
static dVector3 front_left_P;
static dVector3 front_right_P;
static dReal front_frame_position;
static dReal back_frame_position;
dReal PD_torques[MAX_N_ROBOT_PARTS];
static dReal angle_error[MAX_N_ROBOT_PARTS];
static dReal target_angles[8];
static dReal prev_target_angles[8];

dReal front_leg_frame_height;
dReal back_leg_frame_height;
dReal front_leg_frame_pitch;
static dReal initial_front_leg_frame_height;
static dReal initial_back_leg_frame_height;
static dReal prev_front_leg_frame_height;
static dReal prev_back_leg_frame_height;

static dReal front_velocity;
static dReal back_velocity;
static dReal prev_front_velocity =0;
static dReal prev_back_velocity =0;

static dReal new_force;
static dReal new_v_force;
static dReal force;
static dReal v_force;

static int counter = 0; 
GLuint texture[10000];

double frame = 0;

void SaveAsBMP(const char *fileName)
{
    FILE *file;
    unsigned long imageSize;
    GLbyte *data=NULL;
    GLint viewPort[4];
    GLenum lastBuffer;
    BITMAPFILEHEADER bmfh;
    BITMAPINFOHEADER bmih;
    bmfh.bfType='MB';
    bmfh.bfReserved1=0;
    bmfh.bfReserved2=0;
    bmfh.bfOffBits=54;
    glGetIntegerv(GL_VIEWPORT,viewPort);
    imageSize=((viewPort[2]+((4-(viewPort[2]%4))%4))*viewPort[3]*3)+2;
    bmfh.bfSize=imageSize+sizeof(bmfh)+sizeof(bmih);
    data=(GLbyte*)malloc(imageSize);
    glPixelStorei(GL_PACK_ALIGNMENT,4);
    glPixelStorei(GL_PACK_ROW_LENGTH,0);
    glPixelStorei(GL_PACK_SKIP_ROWS,0);
    glPixelStorei(GL_PACK_SKIP_PIXELS,0);
    glPixelStorei(GL_PACK_SWAP_BYTES,1);
    glGetIntegerv(GL_READ_BUFFER,(GLint*)&lastBuffer);
    glReadBuffer(GL_FRONT);
    glReadPixels(0,0,viewPort[2],viewPort[3],GL_BGR_EXT,GL_UNSIGNED_BYTE,data);
    data[imageSize-1]=0;
    data[imageSize-2]=0;
    glReadBuffer(lastBuffer);
    file=fopen(fileName,"wb");
    bmih.biSize=40;
    bmih.biWidth=viewPort[2];
    bmih.biHeight=viewPort[3];
    bmih.biPlanes=1;
    bmih.biBitCount=24;
    bmih.biCompression=0;
    bmih.biSizeImage=imageSize;
	bmih.biXPelsPerMeter=45089;
    bmih.biYPelsPerMeter=45089;
	//bmih.biXPelsPerMeter = bmih.biYPelsPerMeter = 0;
    bmih.biClrUsed=0;
    bmih.biClrImportant=0;
    fwrite(&bmfh,sizeof(bmfh),1,file);
    fwrite(&bmih,sizeof(bmih),1,file);
    fwrite(data,imageSize,1,file);
    free(data);
    fclose(file);
}

void FreeTexture( GLuint texture )
{
  glDeleteTextures( 1, &texture ); 
}

GLuint LoadTexture( const char * filename, int width, int 
height )
{
    GLuint texture;
    unsigned char * data;
    FILE * file;

    //The following code will read in our RAW file
    file = fopen( filename, "rb" );
    if ( file == NULL ) return 0;
    data = (unsigned char *)malloc( width * height * 3 );
    fread( data, width * height * 3, 1, file );
    fclose( file );

    glGenTextures( 1, &texture ); //generate the texture with the loaded data
    glBindTexture( GL_TEXTURE_2D, texture ); //bind the texture to it’s array
	//glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0,  WINDOW_WIDTH, WINDOW_HEIGHT,GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, 
GL_MODULATE ); //set texture environment parameters

    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
 GL_LINEAR_MIPMAP_LINEAR );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
 GL_LINEAR_MIPMAP_LINEAR );


    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 
GL_REPEAT );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 
GL_REPEAT );

    //Generate the texture with mipmaps
    gluBuild2DMipmaps( GL_TEXTURE_2D, 3, width, height, 
GL_RGB, GL_UNSIGNED_BYTE, data ); 
    free( data ); //free the texture
    return texture; //return whether it was successfull
}

void cube (void) {
    glBindTexture( GL_TEXTURE_2D, texture[(int)frame] );
    //glScalef(4,4,1);
    /*glBegin (GL_QUADS);
    glTexCoord2d(0,0); 
    glVertex3f(-1,1,0);
    glTexCoord2d(1,0); 
    glVertex3f(1,1,0);
    glTexCoord2d(1,1); 
    glVertex3f(1,-1,0);
    glTexCoord2d(0,1); 
    glVertex3f(-1,-1,0);
    glEnd();*/

	glBegin (GL_QUADS);
    glTexCoord2d(0,0); 
    glVertex3f(-3.5,-2.5,0);
    glTexCoord2d(1,0); 
    glVertex3f(3.5,-2.5,0);
    glTexCoord2d(1,1); 
    glVertex3f(3.5,2.5,0);
    glTexCoord2d(0,1); 
    glVertex3f(-3.5,2.5,0);
    glEnd();
}

void display (void) {

	if (counter > 999)
	{
    glClearColor (0.0,0.0,0.0,1.0);
    glClear (GL_COLOR_BUFFER_BIT);
    glLoadIdentity(); 
    glEnable( GL_TEXTURE_2D );
    glTranslatef(0,0,-5);
    cube();
	glBindTexture( GL_TEXTURE_2D, texture[(int)frame] );
	glutSwapBuffers(); 
    frame+=30;
   // if (frame > 16)
    {
   // frame = 1;
    }
	//glFlush(); 
	}
}
void reshape (int w, int h) {

	if (counter > 999)
	{
    glViewport (0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective (60, (GLfloat)w / (GLfloat)h, 1.0, 100.0);
    glMatrixMode (GL_MODELVIEW);
	}
}

void init (int limit) {

	char fname[32];
	for (int i=0; i<limit; i++)
	{
		 sprintf(fname,"images/L_%04d.png",i);
		 texture[i] = LoadTexture( fname, WINDOW_WIDTH, WINDOW_HEIGHT );
	}
}


void load_images(int limit)
{
	//glutInit (&argc, argv);
    //glutInitDisplayMode (GLUT_DOUBLE);
    //glutInitWindowSize (WINDOW_WIDTH, WINDOW_HEIGHT);
    //glutInitWindowPosition (100, 100);
    //glutCreateWindow ("A basic OpenGL Window");
    //init(limit);
	
    //glutDisplayFunc (display);
    //glutIdleFunc (display);
		
	//glutReshapeFunc (reshape);
    //glutMainLoop ();
    //return 0;
}


int WindowDump(void)
{
   int i,j;
   FILE *fptr;
   /* This supports animation sequences */
   char fname[32];
   unsigned char *image;

   /* Allocate our buffer for the image */
   if ( (image = (unsigned char *)malloc(3*WINDOW_WIDTH*WINDOW_HEIGHT*sizeof(char))) == NULL) {
      fprintf(stderr,"Failed to allocate memory for image\n");
      return(FALSE);
   }

   glPixelStorei(GL_PACK_ALIGNMENT,1);

   /* Open the file */
   /*if (stereo)
      sprintf(fname,"L_%04d.raw",counter);
   else
      sprintf(fname,"C_%04d.raw",counter);*/

	 sprintf(fname,"images/L_%04d.bmp",counter);
	 SaveAsBMP(fname);
   if ((fptr = fopen(fname,"w")) == NULL) {
      fprintf(stderr,"Failed to open file for window dump\n");
      return(FALSE);
   }

   /* Copy the image into our buffer */
   glReadBuffer(GL_BACK_LEFT);
   glReadPixels(0,0,WINDOW_WIDTH,WINDOW_HEIGHT,GL_RGB,GL_UNSIGNED_BYTE,image);

   /* Write the raw file */
   /* fprintf(fptr,"P6\n%d %d\n255\n",width,height); for ppm */
   for (j=WINDOW_HEIGHT-1;j>=0;j--) {
      for (i=0;i<WINDOW_WIDTH;i++) {
         fputc(image[3*j*WINDOW_WIDTH+3*i+0],fptr);
         fputc(image[3*j*WINDOW_WIDTH+3*i+1],fptr);
         fputc(image[3*j*WINDOW_WIDTH+3*i+2],fptr);
      }
   }
   fclose(fptr);

  // if (stereo) {
      /* Open the file */
      sprintf(fname,"images/R_%04d.bmp",counter);
      if ((fptr = fopen(fname,"w")) == NULL) {
         fprintf(stderr,"Failed to open file for window dump\n");
         return(FALSE);
      }
		 SaveAsBMP(fname);

      /* Copy the image into our buffer */
      glReadBuffer(GL_BACK_RIGHT);
      glReadPixels(0,0,WINDOW_WIDTH,WINDOW_HEIGHT,GL_RGB,GL_UNSIGNED_BYTE,image);

      /* Write the raw file */
      /* fprintf(fptr,"P6\n%d %d\n255\n",width,height); for ppm */
      for (j=WINDOW_HEIGHT-1;j>=0;j--) {
         for (i=0;i<WINDOW_WIDTH;i++) {
            fputc(image[3*j*WINDOW_WIDTH+3*i+0],fptr);
            fputc(image[3*j*WINDOW_WIDTH+3*i+1],fptr);
            fputc(image[3*j*WINDOW_WIDTH+3*i+2],fptr);
         }
      }
      fclose(fptr);
   //}

   /* Clean up */
   counter++;
   free(image);
   return(TRUE);
}



void dReal_to_vector(const dReal* source, dVector3 target)
{
	target[0] = source[0];
	target[1] = source[1];
	target[2] = source[2];
}

void balance_control()
{
	dReal force1;
	const dReal* pos = dBodyGetPosition(robot_bodies[12]); 
	dReal temp_pos = pos[1];
	
	dVector3 front_left;
	dVector3 front_right; 
	dBodyGetRelPointPos(robot_bodies[0], FOOT_LENGTH/2, 0, 0, front_left);
	dBodyGetRelPointPos(robot_bodies[2], FOOT_LENGTH/2, 0, 0, front_right);
	
	force1 = (front_leg_frame_height  - initial_front_leg_frame_height)*Kp_force*10;

	dBodyAddForce(robot_bodies[12], 0, 0, -force1);

	if (temp_pos !=0)
	{
		force1  = pos[1]*Kp_force*200;
		dBodyAddForce(robot_bodies[12], 0, -force1, 0);
	}

	pos = dBodyGetPosition(robot_bodies[17]);
	temp_pos = pos[1];

	
	dVector3 back_left;
	dVector3 back_right; 
	dBodyGetRelPointPos(robot_bodies[1], FOOT_LENGTH/2, 0, 0, back_left);
	dBodyGetRelPointPos(robot_bodies[3], FOOT_LENGTH/2, 0, 0, back_right);

	force1 = (back_leg_frame_height  - initial_back_leg_frame_height)*Kp_force*20;
		
	dBodyAddForce(robot_bodies[17], 0, 0, -force1);

	if (temp_pos !=0)
	{
		force1  = pos[1]*Kp_force*200;
		dBodyAddForce(robot_bodies[17], 0, -force1, 0);
	}

}

void apply_virtual_forces_front_new()
{
	int n = 0;
	if (front_left_phase == 0)
		n++;

	if (front_right_phase == 0)
		n++;

	force =  Kp_force*(initial_front_leg_frame_height - front_leg_frame_height) - Kd_force*(front_leg_frame_height - prev_front_leg_frame_height)/(TIME_STEP*5*2) + 350;
	prev_front_leg_frame_height = front_leg_frame_height;
			

	printf("front velocity is %f\n", force);
	if (n < 2)
	{
	
		const dReal* new_frame_position = dBodyGetPosition(robot_bodies[12]);
		front_velocity = (new_frame_position[0] - front_frame_position)/(TIME_STEP*5);
		v_force = Kp_velocity*(velocity - front_velocity) - Kd_velocity*(front_velocity - prev_front_velocity)/(TIME_STEP*5*2);
		prev_front_velocity = front_velocity;
		front_frame_position = new_frame_position[0];

	}
	

	if (n>0)
	{
		new_force = force/n;
		new_v_force = v_force/n;

		if (front_right_phase != 0)
		{

			for(int i=12;i<15;i++)
			{
				dBodyAddForce(robot_bodies[i], 0, 0, new_force);
				dBodyAddForce(robot_bodies[i], new_v_force, 0, 0);
			}
		}
		else
		{
			if (new_v_force>0)
			{
				const dReal* temp_frame_position = dBodyGetPosition(robot_bodies[12]);
				const dReal* temp_foot_position = dBodyGetPosition(robot_bodies[2]);
				dReal diff = temp_frame_position[0] - temp_foot_position[0];
				printf("diff is %f\n", diff);
				if (diff > 0.1)
				{
					dBodySetForce(robot_bodies[2], 0, 0, -10000);
				}
				else
					dBodyAddForce(robot_bodies[2], -new_v_force/20, 0, 0);
					
			}
			
			
			//dBodyAddForce(robot_bodies[2], 0, 0, -new_force*50);
			dBodyAddForce(robot_bodies[2], 0, 0, -10000);

			
		}

		if (front_left_phase != 0)
		{
			//dBodyAddForce(robot_bodies[2], -2*new_v_force, 0, -2*new_force);
			//dBodyAddForce(robot_bodies[2], -new_v_force*0.75, 0,  -2*new_force);


			//dBodyAddForce(robot_bodies[2], 0, 0, -new_force);

			for(int i=12;i<15;i++)
			{
				dBodyAddForce(robot_bodies[i], 0, 0, new_force);
				//dBodyAddForceAtRelPos(robot_bodies[i], 0, 0, new_force, 0, BOX_WIDTH/2, 0);
				dBodyAddForce(robot_bodies[i], new_v_force, 0, 0);
			}

			
		}
		else
		{

			if (new_v_force>0)
			{
				const dReal* temp_frame_position = dBodyGetPosition(robot_bodies[12]);
				const dReal* temp_foot_position = dBodyGetPosition(robot_bodies[0]);
				dReal diff = temp_frame_position[0] - temp_foot_position[0];
				printf("diff is %f\n", diff);
				if (diff > 0.1)
					dBodySetForce(robot_bodies[0], 0, 0, -10000);
					//dBodyAddForce(robot_bodies[0], 0, 0, -1000);
				else
					dBodyAddForce(robot_bodies[0], -new_v_force/20, 0, 0);
					
			}
			
			//dBodyAddForce(robot_bodies[0], 0, 0, -new_force*50);
			dBodyAddForce(robot_bodies[0], 0, 0, -10000);

		}
	}

}


void apply_virtual_forces_back_new()
{
	int n = 0;
	if (back_left_phase == 0)
		n++;

	if (back_right_phase == 0)
		n++;

	force = Kp_force*(initial_back_leg_frame_height - back_leg_frame_height) - Kd_force*(back_leg_frame_height - prev_back_leg_frame_height)/(TIME_STEP*5*2) + 350;
	prev_back_leg_frame_height = back_leg_frame_height;
	
	printf("back velocity is %f\n", back_velocity);
	if (n < 2)
	{
	
	//const dReal* vel = dBodyGetLinearVel(robot_bodies[12]);
	const dReal* new_frame_position = dBodyGetPosition(robot_bodies[17]);
	back_velocity = (new_frame_position[0] - back_frame_position)/(TIME_STEP*5);
	v_force = Kp_velocity*(velocity - back_velocity) - Kd_velocity*(back_velocity - prev_back_velocity)/(TIME_STEP*10);
	prev_back_velocity = back_velocity;
	back_frame_position = new_frame_position[0];

	}
	
	

	if (n>0)
	{
		new_force = force/n;
		new_v_force = v_force/n;
		
		//printf("back force is %f\n", new_force);

		if (back_right_phase != 0)
		{
			//dBodyAddForce(robot_bodies[1], -2*new_v_force, 0, -2*new_force);
			
			//dBodyAddForce(robot_bodies[1], 0, 0, -new_force);
			for(int i=15;i<18;i++)
			{
				dBodyAddForce(robot_bodies[i], 0, 0, 3*new_force);
				//dBodyAddForceAtRelPos(robot_bodies[i], 0, 0, 2*new_force, 0, -BOX_WIDTH/2, 0);
				dBodyAddForce(robot_bodies[i], 2*new_v_force, 0, 0);
			}
		}
		else
		{
			if (new_v_force>0)
			{
				const dReal* temp_frame_position = dBodyGetPosition(robot_bodies[17]);
				const dReal* temp_foot_position = dBodyGetPosition(robot_bodies[3]);
				dReal diff = temp_frame_position[0] - temp_foot_position[0];
				printf("diff is %f\n", diff);
				if (diff > 0.1)
					dBodySetForce(robot_bodies[3], 0, 0, -10000);
					//dBodyAddForce(robot_bodies[3], 0, 0, -1000);
				else
					dBodyAddForce(robot_bodies[3], -new_v_force/5, 0, 0);
					
			}
			
				//dBodyAddForce(robot_bodies[3], 0, 0, -new_force*50);
				dBodyAddForce(robot_bodies[3], 0, 0, -10000);
		}

		if (back_left_phase != 0)
		{
			//dBodyAddForce(robot_bodies[3], -2*new_v_force, 0, -2*new_force);
			
			
			//dBodyAddForce(robot_bodies[3], 0, 0, -new_force);
			for(int i=15;i<18;i++)
			{
				dBodyAddForce(robot_bodies[i], 0, 0, 3*new_force);
				//dBodyAddForceAtRelPos(robot_bodies[i], 0, 0, 2*new_force, 0, BOX_WIDTH/2, 0);
				dBodyAddForce(robot_bodies[i], 2*new_v_force, 0, 0);
			}
		}
		else
		{
			//if ((back_left_phase < 0.7)&&(new_v_force>0))
			if (new_v_force>0)
			{
				const dReal* temp_frame_position = dBodyGetPosition(robot_bodies[17]);
				const dReal* temp_foot_position = dBodyGetPosition(robot_bodies[1]);
				dReal diff = temp_frame_position[0] - temp_foot_position[0];
				printf("diff is %f\n", diff);
				if (diff > 0.1)
					dBodySetForce(robot_bodies[1], 0, 0, -10000);
					//dBodyAddForce(robot_bodies[1], 0, 0, -1000);
				else
					dBodyAddForce(robot_bodies[1], -new_v_force/5, 0, 0);
					
			}
			
				//dBodyAddForce(robot_bodies[1], 0, 0, -new_force*50);
				dBodyAddForce(robot_bodies[1], 0, 0, -10000);
			
		}
	}

}

dReal tune(dReal num)
{
	if (num > 1.0)
		num = 1;
	if (num < -1)
		num = -1;

	return num;
}

void calculate_angle(const dReal* source, dReal* target,dReal *theta2,dReal *theta1)
{
   dReal x = (2*SHANK_HEIGHT-FOOT_THICKNESS) - target[2];
   dReal y = source[0] - target[0];
   dReal l1 = SHANK_HEIGHT-FOOT_THICKNESS/2;
   dReal l2 = SHANK_HEIGHT-FOOT_THICKNESS/2;
   dReal temp_var = x/sqrt(x*x+y*y);
   temp_var = tune(temp_var);
   dReal theta_temp = acos(temp_var);
   temp_var = (l1*l1 + x*x + y*y - l2*l2)/(2*l1*sqrt(x*x + y*y));
   temp_var = tune(temp_var);
   dReal angle1 = theta_temp - acos(temp_var);
   temp_var = (l1*l1 + l2*l2 - x*x - y*y)/(2*l1*l2);
   temp_var = tune(temp_var);
   dReal angle2 = PI- acos(temp_var);
	
	*theta1 = angle1;
	*theta2 = angle2;

}

void apply_IK()
{
		calculate_angle(Front_left_P1, front_left_P, &target_angles[0], &target_angles[1]);
		calculate_angle(Back_left_P1, back_left_P, &target_angles[2], &target_angles[3]);
		calculate_angle(Front_right_P1, front_right_P, &target_angles[4], &target_angles[5]);
		calculate_angle(Back_right_P1, back_right_P, &target_angles[6], &target_angles[7]);
}

void get_P2(dVector3 P1, const dReal *frame, dVector3 P2, int n)
{
	if (n < 0 )
	{
		
			P2[0] = frame[0] + 0.1 + FOOT_LENGTH/2;
	}
	else
	{

		//if (walk_start_front==0)
		{
			P2[0] = frame[0] + 0.1 + FOOT_LENGTH/2;
		}
		
	}
	P2[1] = P1[1];
	P2[2] = P1[2];
}

void get_P(const dReal* P1, dVector3 P2, dReal time, dVector3 P)
{
	P[0] = (1-time)*P1[0] + time*P2[0]; 
	P[1] = P1[1];
	P[2] = P1[2];
}


dReal get_swing_foot_timing(dReal phase)
{
	return phase;
}

dReal get_back_swing_foot_height(dReal phase)
{
	//if (walk_start_back == 0)
	//	return (FOOT_THICKNESS*0.25 -((phase-0.5)*(phase-0.5)*0.04));
	/*else if (walk_start_back == 1)
		return (FOOT_THICKNESS*0.6-((phase-0.5)*(phase-0.5)*0.096));*/
	//else
		return (FOOT_THICKNESS*0.75-((phase-0.5)*(phase-0.5)*0.12));

}
dReal get_front_swing_foot_height(dReal phase)
{
	
	//if (walk_start_front == 0)
	//	return (FOOT_THICKNESS*0.25 -((phase-0.5)*(phase-0.5)*0.04));
	/*else if (walk_start_front == 1)
		return (FOOT_THICKNESS*0.25 -((phase-0.5)*(phase-0.5)*0.04));*/
	//else
		return (FOOT_THICKNESS*0.75 -((phase-0.5)*(phase-0.5)*0.12));
		
}
dReal get_front_leg_frame_pitch()
{
	const dReal *rot;
	rot = dBodyGetRotation(robot_bodies[12]);
	return rot[3];
}

dReal get_back_leg_frame_height()
{
	const dReal* pos;
	dReal height;
	pos = dBodyGetPosition(robot_bodies[17]);
	height = pos[2];
	return height;
}

dReal get_front_leg_frame_height()
{
	const dReal* pos;
	dReal height;
	pos = dBodyGetPosition(robot_bodies[12]);
	height = pos[2];
	return height;
}

void update_leg_phase()
{
	Back_right_P1 = dBodyGetPosition(robot_bodies[3]);
	Front_right_P1 = dBodyGetPosition(robot_bodies[2]);
	Front_left_P1 = dBodyGetPosition(robot_bodies[0]);
	Back_left_P1 = dBodyGetPosition(robot_bodies[1]);


	if(stride_phase>0.55 && stride_phase<0.95)
	{
		Back_right_P1 = dBodyGetPosition(robot_bodies[3]);
		dReal_to_vector(Back_right_P1, back_right_P1);
		get_P2(back_right_P1, dBodyGetPosition(robot_bodies[17]),back_right_P2, -1);
		back_right_phase = 0.0;		
	}
	else
	{
		back_right_phase = back_right_phase + TIME_STEP*5*1.67*2;
		if (back_right_phase >= 1.00)
			back_right_phase = 0.0;
	}
	if(stride_phase>0.05 && stride_phase<0.45)
	{
		Front_right_P1 = dBodyGetPosition(robot_bodies[2]);
		dReal_to_vector(Front_right_P1, front_right_P1);
		get_P2(front_right_P1, dBodyGetPosition(robot_bodies[12]),front_right_P2, 1);
		front_right_phase = 0.0;
		
	}
	else
	{
		front_right_phase = front_right_phase + TIME_STEP*5*1.67*2;
		if (front_right_phase >= 1.00)
			front_right_phase = 0.0;
		
	}
	if(stride_phase>0.55 && stride_phase<0.95)
	{
		Front_left_P1 = dBodyGetPosition(robot_bodies[0]);
		dReal_to_vector(Front_left_P1, front_left_P1);
		get_P2(front_left_P1, dBodyGetPosition(robot_bodies[12]),front_left_P2,1);
		front_left_phase = 0.0;	
	}
	else
	{
		front_left_phase = front_left_phase + TIME_STEP*5*1.67*2;
		if (front_left_phase >= 1.00)
			front_left_phase = 0.0;
		
	}
	if(stride_phase>0.05 && stride_phase<0.45)
	{
		Back_left_P1 = dBodyGetPosition(robot_bodies[1]);
		dReal_to_vector(Back_left_P1, back_left_P1);
		get_P2(back_left_P1, dBodyGetPosition(robot_bodies[17]),back_left_P2,-1);
		back_left_phase = 0.0;
	
	}
	else
	{
		back_left_phase = back_left_phase + TIME_STEP*5*1.67*2;
		if (back_left_phase >= 1.00)
			back_left_phase = 0.0;	
	}

	if(stride_phase > 0.1)
		walk_start_back = 1;

	if(stride_phase > 0.35)
		walk_start_front = 1;

	if(stride_phase > 0.7)
		walk_start_back = 2;

	if(stride_phase > 0.9)
		walk_start_front = 2;

	if ((back_left_phase > 0.91)&&(stride_phase>0.9))
	{
		stride_phase = 0.0;
	}

	if(stride_phase == 0.5)
	{
		/*if(initial_front_leg_frame_height > 0.28)
		{
			initial_front_leg_frame_height = initial_front_leg_frame_height - 0.01;
			initial_back_leg_frame_height = initial_back_leg_frame_height - 0.01;
		}*/
	}
}


void compute_pd_torques()
{
	front_leg_frame_height = get_front_leg_frame_height();
	back_leg_frame_height = get_back_leg_frame_height();
	front_leg_frame_pitch = get_front_leg_frame_pitch();

	get_P(back_left_P1, back_left_P2, back_left_phase, back_left_P);
	get_P(back_right_P1, back_right_P2, back_right_phase, back_right_P);
	get_P(front_left_P1, front_left_P2, front_left_phase, front_left_P);
	get_P(front_right_P1, front_right_P2, front_right_phase, front_right_P);

	back_left_P[2] = get_back_swing_foot_height(back_left_phase);
	back_right_P[2] = get_back_swing_foot_height(back_right_phase);
	front_left_P[2] = get_front_swing_foot_height(front_left_phase);
	front_right_P[2] = get_front_swing_foot_height(front_right_phase);

	dReal temp_angle_new =0;
	apply_IK();

	dReal angle_rate;

	for(int i=4; i<8; i++)
	{
		temp_angle_new = dJointGetHingeAngle(robot_joints[i]);
		angle_error[i] = prev_target_angles[(i-4)*2] - temp_angle_new;
		angle_rate = dJointGetHingeAngleRate(robot_joints[i]);
		PD_torques[i] = Kp*(target_angles[(i-4)*2] + angle_error[i]) - Kd*angle_rate;
		prev_target_angles[(i-4)*2] = target_angles[(i-4)*2];
	}

	for(int i=20; i<24; i++)
	{
		temp_angle_new = dJointGetHingeAngle(robot_joints[i]);
		angle_error[i] = prev_target_angles[(i-20)*2+1] - temp_angle_new;	
		PD_torques[i] = Kp*(target_angles[(i-20)*2+1] + angle_error[i]) - Kd*dJointGetHingeAngleRate(robot_joints[i]);
		prev_target_angles[(i-20)*2+1] = target_angles[(i-20)*2+1];
	}

}

void trot()
{

	update_leg_phase();


	compute_pd_torques();

	if (front_left_phase != 0)
	{
		dJointAddHingeTorque (robot_joints[4], PD_torques[4] );	
			dJointAddHingeTorque (robot_joints[20], PD_torques[20] );	
	}
	if (back_left_phase != 0)
	{
		dJointAddHingeTorque (robot_joints[5], PD_torques[5] );	
			dJointAddHingeTorque (robot_joints[21], PD_torques[21] );	
	}
	if (front_right_phase != 0)
	{
		dJointAddHingeTorque (robot_joints[6], PD_torques[6] );	
			dJointAddHingeTorque (robot_joints[22], PD_torques[22] );	
	
	}
	if (back_right_phase != 0)
	{
		dJointAddHingeTorque (robot_joints[7], PD_torques[7] );	
			dJointAddHingeTorque (robot_joints[23], PD_torques[23] );	
		
	}
	
	printf("back left stride phase is %f\n", back_left_phase);
	printf("front left stride phase is %f\n", front_left_phase);
	printf("back right stride phase is %f\n", back_right_phase);
	printf("front right stride phase is %f\n", front_right_phase);
	
	/*printf("back left initial P1 is %f\n", back_left_P1[0]);
	printf("back left P1 is %f\n", Back_left_P1[0]);
	printf("back left P2 is %f\n", back_left_P2[0]);
	printf("back left P is %f\n", back_left_P[0]);
	*/
	/*
	printf("back left P1 is %f\n", back_left_P1[0]);
	printf("back right P1 is %f\n", back_right_P1[0]);
	printf("front left P1 is %f\n", front_left_P1[0]);
	printf("front right P1 is %f\n", front_right_P1[0]);

	printf("back left P2 is %f\n", back_left_P2[0]);
	printf("back right P2 is %f\n", back_right_P2[0]);
	printf("front left P2 is %f\n", front_left_P2[0]);
	printf("front right P2 is %f\n", front_right_P2[0]);

	printf("back left P is %f\n", back_left_P[0]);
	printf("back right P is %f\n", back_right_P[0]);
	printf("front left P is %f\n", front_left_P[0]);
	printf("front right P is %f\n", front_right_P[0]);

	*/

	apply_virtual_forces_front_new();
	apply_virtual_forces_back_new();
	
	balance_control();

	stride_phase = stride_phase + TIME_STEP*5*2;

}


static void nearCallback2( void *data, dGeomID a, dGeomID b )
{
  const unsigned max_contacts = 4;
    dContact contacts[max_contacts];
    
    if (!dGeomGetBody(a) && !dGeomGetBody(b))
        return; // don't handle static geom collisions

    int n = dCollide(a, b, max_contacts, &contacts[0].geom, sizeof(dContact));


 for (int i=0; i<n; ++i) {
        contacts[i].surface.mode = dContactBounce | dContactApprox1 | dContactSoftERP;
        contacts[i].surface.mu = 10;
        contacts[i].surface.bounce = 0.2;
        contacts[i].surface.bounce_vel = 0;
        contacts[i].surface.soft_erp = 1e-3;
        //clog << "depth: " << contacts[i].geom.depth << endl;


        dJointID contact = dJointCreateContact(world, contact_joint_group, &contacts[i]);
        dJointAttach(contact, dGeomGetBody(a), dGeomGetBody(b));

        /*dMatrix3 r;
        dRSetIdentity(r);
        dsSetColor(0, 0, 1);
        dsSetTexture(DS_NONE);
        dsDrawSphere(contacts[i].geom.pos, r, 0.01);
        dsSetColor(0, 1, 0);
        dVector3 pos2;
        dAddScaledVectors3(pos2, contacts[i].geom.pos, contacts[i].geom.normal, 1, 0.1);
        dsDrawLine(contacts[i].geom.pos, pos2);*/

    }
}



void draw_stuff(dGeomID g)
{
  dReal length;
  dReal radius;

  int gclass = dGeomGetClass(g);
  const dReal *pos = dGeomGetPosition(g);
  const dReal *rot = dGeomGetRotation(g);

  switch (gclass){
	  case dBoxClass:
        {
            dVector3 lengths;
            //dsSetColorAlpha(1, 1, 0, 0.5);
            dGeomBoxGetLengths(g, lengths);
			dsSetColor (1,0.5,0);
            dsDrawBox(pos, rot, lengths);
            break;
        }
	  case dCylinderClass:
		{
			dGeomCylinderGetParams(g, &radius, &length);
			dsSetColor (1,0.5,0);
			dsDrawCylinder( pos, rot, length, radius );
			break;
		  }
	 case dCapsuleClass:
		{
			dGeomCapsuleGetParams(g, &radius, &length);
			dsSetColor (1,0.5,0);
			dsDrawCapsule( pos, rot, length, radius );
			break;
		  }

		default:
        {}
  }	
}

// simulation loop
static void simLoop (int pause)
{
  int i;

  trot();

  // find collisions and add contact joints
  dSpaceCollide( space, 0, &nearCallback2  );


  // step the simulation

	dWorldStep( world, TIME_STEP);  

	
  // increment time
  time += TIME_STEP*5;
  walk_time += TIME_STEP*5;

  // remove all contact joints
  dJointGroupEmpty( contact_joint_group );

  // now we draw everything
  unsigned ngeoms = dSpaceGetNumGeoms(space);
  for (unsigned i=0; i<ngeoms; ++i) {
        dGeomID g = dSpaceGetGeom(space, i);

        if (dGeomGetClass(g) == 4)
            continue; // drawstuff is already drawing it for us

		char fname[32];

		draw_stuff(g);

		/*
		//WindowDump();
		sprintf(fname,"images/L_%04d.png",counter);
		SaveAsBMP(fname);
		texture[counter] = LoadTexture( fname, WINDOW_WIDTH, WINDOW_HEIGHT );	
		counter++;
				
		if (counter == 1000)
		{
			 //glutInit (&argc, argv);
			 glutInitDisplayMode (GLUT_DOUBLE);
			 glutInitWindowSize (WINDOW_WIDTH, WINDOW_HEIGHT);
			 glutInitWindowPosition (100, 100);
			glutCreateWindow ("A basic OpenGL Window");
			//init(limit);
			
		}
		if (counter > 1000) 
		{
			
		//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//		load_images(counter);
	//		counter++;
			glutDisplayFunc (display);
			glutIdleFunc (display);		
			glutReshapeFunc (reshape);
			//glutMainLoop ();
			counter++;
			
		}

		*/
    }
  
}

static void start()
{
	//view left
	// float xyz[3] = { 1.0f, 2.3f, 1.0f };
	//float hpr[3] = { -110.0f, -7.0f, 0.0f };

	
  //view right
  float xyz[3] = { 1.0f, -2.3, 1.0f };
  float hpr[3] = { 110.0f, 3.0f, 0.0f };


	//view centre
  //float xyz[3] = { 2.0f, 0.0f, 1.0f };
  //float hpr[3] = { 180.0f, 0.0f, 0.0f };

	//view back
  //float xyz[3] = { -2.0f, 0.0f, 1.0f };
  //float hpr[3] = { 0.0f, 0.0f, 0.0f };

  dsSetViewpoint( xyz, hpr );
}

void attach_joints()
{
	int i;
	for (i=0; i<4; i++) {
    robot_joints[i] = dJointCreateHinge (world,0);
    dJointAttach (robot_joints[i],robot_bodies[i],robot_bodies[i+4]);
    const dReal *a = dBodyGetPosition (robot_bodies[i]);
    dJointSetHingeAnchor (robot_joints[i],a[0]-FOOT_LENGTH/2+SHANK_RADIUS,a[1], a[2]);
    dJointSetHingeAxis (robot_joints[i],0,1,0);
  }

	for (i=4; i<8; i++) {
    robot_joints[i] = dJointCreateHinge (world,0);
    dJointAttach (robot_joints[i],robot_bodies[i],robot_bodies[i+4]);
    const dReal *a = dBodyGetPosition (robot_bodies[i]);
	dJointSetHingeAnchor (robot_joints[i],a[0],a[1],SHANK_HEIGHT - FOOT_THICKNESS/2 );
	dJointSetHingeAxis (robot_joints[i],0,1,0);
  }

	for (i=12; i<17; i++) {
    robot_joints[i-4] = dJointCreateHinge (world,0);
    dJointAttach (robot_joints[i-4],robot_bodies[i],robot_bodies[i+1]);
    const dReal *a = dBodyGetPosition (robot_bodies[i]);
    dJointSetHingeAnchor (robot_joints[i-4],a[0]-BOX_LENGTH/2,a[1],a[2]);
    dJointSetHingeAxis (robot_joints[i-4],0,1,0);
  }

	for (i=8; i<12; i++) {

    robot_joints[i+12] = dJointCreateHinge (world,0);
    dJointAttach (robot_joints[i+12],robot_bodies[i],robot_bodies[i+13]);
    const dReal *a = dBodyGetPosition (robot_bodies[i]);
	dJointSetHingeAnchor (robot_joints[i+12],a[0],a[1],2*SHANK_HEIGHT - FOOT_THICKNESS/2 );
	dJointSetHingeAxis (robot_joints[i+12],0,1,0);

	
  }

	robot_joints[18] = dJointCreateHinge (world,0);
    dJointAttach (robot_joints[18],robot_bodies[12],robot_bodies[19]);
    const dReal *a = dBodyGetPosition (robot_bodies[12]);
    dJointSetHingeAnchor (robot_joints[18],a[0],a[1],a[2]);
    dJointSetHingeAxis (robot_joints[18],0,1,0);

	robot_joints[14] = dJointCreateFixed(world,0);
	dJointAttach(robot_joints[14], robot_bodies[21], robot_bodies[12]);
	dJointSetFixed (robot_joints[14]);

	robot_joints[15] = dJointCreateFixed(world,0);
	dJointAttach(robot_joints[15], robot_bodies[22], robot_bodies[17]);
	dJointSetFixed (robot_joints[15]);

	robot_joints[16] = dJointCreateFixed(world,0);
	dJointAttach(robot_joints[16], robot_bodies[23], robot_bodies[12]);
	dJointSetFixed (robot_joints[16]);

	robot_joints[17] = dJointCreateFixed(world,0);
	dJointAttach(robot_joints[17], robot_bodies[24], robot_bodies[17]);
	dJointSetFixed (robot_joints[17]);

	robot_joints[18] = dJointCreateFixed(world,0);
	dJointAttach(robot_joints[18], robot_bodies[19], robot_bodies[20]);
	dJointSetFixed (robot_joints[18]);

	robot_joints[19] = dJointCreateFixed(world,0);
	dJointAttach(robot_joints[19], robot_bodies[20], robot_bodies[18]);
	dJointSetFixed (robot_joints[19]);



	for (i=24; i<31; i++) {
    robot_joints[i] = dJointCreateHinge (world,0);
    dJointAttach (robot_joints[i],robot_bodies[i+1],robot_bodies[i+2]);
    const dReal *a = dBodyGetPosition (robot_bodies[i+1]);
    dJointSetHingeAnchor (robot_joints[i],a[0]-TAIL_LENGTH/2,a[1],a[2]);
    dJointSetHingeAxis (robot_joints[i],0,1,0);
  }

	robot_joints[31] = dJointCreateHinge (world,0);
    dJointAttach (robot_joints[31],robot_bodies[25],robot_bodies[17]);
    const dReal *g = dBodyGetPosition (robot_bodies[17]);
    dJointSetHingeAnchor (robot_joints[31],g[0],g[1],g[2]);
    dJointSetHingeAxis (robot_joints[31],0,1,0);


}

void create_bodies_and_geoms()
{

  int i, j;
  dMass m;

	// FOOT
  for (i=0; i<4; i++) {
    robot_bodies[i] = dBodyCreate (world);
    dMassSetBox ( &m, 1, FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS);
    dMassAdjust (&m,FOOT_MASS);
    dBodySetMass (robot_bodies[i],&m);
    robot_geoms[i] = dCreateBox (space, FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS);
    dGeomSetBody (robot_geoms[i],robot_bodies[i]);
  } 

  dBodySetPosition (robot_bodies[0],LEG_X+SHANK_RADIUS/2, LEG_Y, FOOT_Z );
  dBodySetPosition (robot_bodies[1],-LEG_X+SHANK_RADIUS/2, LEG_Y2, FOOT_Z);
  dBodySetPosition (robot_bodies[2],LEG_X+SHANK_RADIUS/2, -LEG_Y, FOOT_Z);
  dBodySetPosition (robot_bodies[3],-LEG_X+SHANK_RADIUS/2, -LEG_Y2, FOOT_Z);


  // SHANK
	for (i=4; i<8; i++) {
	robot_bodies[i] = dBodyCreate (world);
    dMassSetCylinder (&m,1,1,SHANK_RADIUS,SHANK_HEIGHT );
    dMassAdjust (&m,SHANK_MASS);
    dBodySetMass (robot_bodies[i],&m);
    robot_geoms[i] = dCreateCylinder (space,SHANK_RADIUS, SHANK_HEIGHT);
    dGeomSetBody (robot_geoms[i],robot_bodies[i]);
  }




  dBodySetPosition (robot_bodies[4],LEG_X, LEG_Y, SHANK_Z);
  dBodySetPosition (robot_bodies[5],-LEG_X, LEG_Y2, SHANK_Z);
  dBodySetPosition (robot_bodies[6],LEG_X, -LEG_Y, SHANK_Z);
  dBodySetPosition (robot_bodies[7],-LEG_X, -LEG_Y2, SHANK_Z);

  // THIGH
	for (i=8; i<12; i++) {
    robot_bodies[i] = dBodyCreate (world);
    dMassSetCylinder (&m,1,1,SHANK_RADIUS,SHANK_HEIGHT );
    dMassAdjust (&m,THIGH_MASS);
    dBodySetMass (robot_bodies[i],&m);
    robot_geoms[i] = dCreateCylinder (space,SHANK_RADIUS, SHANK_HEIGHT);
    dGeomSetBody (robot_geoms[i],robot_bodies[i]);
  }
  dBodySetPosition (robot_bodies[8],LEG_X, LEG_Y,THIGH_Z);
  dBodySetPosition (robot_bodies[9],-LEG_X,LEG_Y2,THIGH_Z);
  dBodySetPosition (robot_bodies[10],LEG_X,-LEG_Y,THIGH_Z);
  dBodySetPosition (robot_bodies[11],-LEG_X,-LEG_Y2,THIGH_Z);

  double temp_x = TORSO_X;
  double temp_length = BOX_LENGTH;
  double temp_width = BOX_WIDTH;
  double temp_thickness = BOX_THICKNESS;
  double temp_mass = TORSO_BOX_MASS;

	  //TORSO
	for (i=12; i<18; i++) {
    robot_bodies[i] = dBodyCreate (world);
    dMassSetBox (&m,1,temp_length,temp_width,temp_thickness);
    dMassAdjust (&m,temp_mass);
    dBodySetMass (robot_bodies[i],&m);
    robot_geoms[i] = dCreateBox (space,temp_length,temp_width,temp_thickness);
    dGeomSetBody (robot_geoms[i],robot_bodies[i]);
	if (i<15)
		dBodySetPosition (robot_bodies[i],temp_x,TORSO_Y,TORSO_Z);
	else
		dBodySetPosition (robot_bodies[i],temp_x,TORSO_Y,TORSO_Z);

	temp_x = temp_x - BOX_LENGTH + 0.001;
	//temp_width = temp_width - BOX_WIDTH/7;
	temp_thickness = temp_thickness - BOX_THICKNESS/10;
	temp_mass = temp_mass - TORSO_BOX_MASS/7;

  }

	//head and neck
	robot_bodies[19] = dBodyCreate (world);
    dMassSetBox (&m,1,BOX_LENGTH*6/7,BOX_WIDTH,BOX_THICKNESS);
    dMassAdjust (&m,temp_mass);
    dBodySetMass (robot_bodies[19],&m);
    robot_geoms[19] = dCreateBox (space,BOX_LENGTH*6/7,BOX_WIDTH,BOX_THICKNESS);
    dGeomSetBody (robot_geoms[19],robot_bodies[19]);
	dBodySetPosition (robot_bodies[19],TORSO_X + BOX_LENGTH/2,TORSO_Y,TORSO_Z+FOOT_THICKNESS);
	dMatrix3 R;
	dRFromAxisAndAngle (R,0,1,0,-0.7);
	dBodySetRotation (robot_bodies[19], R);

	robot_bodies[20] = dBodyCreate (world);
    dMassSetCylinder (&m,1,1,NECK_RADIUS,NECK_HEIGHT );
    dMassAdjust (&m,NECK_MASS);
    dBodySetMass (robot_bodies[20],&m);
    robot_geoms[20] = dCreateCylinder (space,NECK_RADIUS, NECK_HEIGHT);
    dGeomSetBody (robot_geoms[20],robot_bodies[20]);
	dBodySetPosition (robot_bodies[20],NECK_X,NECK_Y,NECK_Z);
	dMatrix3 R1;
	dRFromAxisAndAngle (R1,0,1,0,-2.5);
	dBodySetRotation (robot_bodies[20], R1);

	robot_bodies[18] = dBodyCreate (world);
    dMassSetCapsule (&m,1,1,HEAD_RADIUS,HEAD_HEIGHT );
    dMassAdjust (&m,HEAD_MASS);
    dBodySetMass (robot_bodies[18],&m);
    robot_geoms[18] = dCreateCapsule (space,HEAD_RADIUS, HEAD_HEIGHT);
    dGeomSetBody (robot_geoms[18],robot_bodies[18]);
	dBodySetPosition (robot_bodies[18],HEAD_X,HEAD_Y,HEAD_Z);
	dMatrix3 R2;
	dRFromAxisAndAngle (R2,0,1,0,-0.9);
	dBodySetRotation (robot_bodies[18], R2);

	 // THIGH JOINT
	for (i=21; i<25; i++) {
    robot_bodies[i] = dBodyCreate (world);
    dMassSetCylinder (&m,1,1,SHANK_RADIUS,THIGH_JOINT_HEIGHT );
    dMassAdjust (&m,THIGH_MASS);
    dBodySetMass (robot_bodies[i],&m);
    robot_geoms[i] = dCreateCylinder (space,SHANK_RADIUS, THIGH_JOINT_HEIGHT);
    dGeomSetBody (robot_geoms[i],robot_bodies[i]);
	 }
	 dBodySetPosition (robot_bodies[21],LEG_X, LEG_Y,THIGH_JOINT_Z);
	 dBodySetPosition (robot_bodies[22],-LEG_X,LEG_Y2,THIGH_JOINT_Z);
	 dBodySetPosition (robot_bodies[23],LEG_X,-LEG_Y,THIGH_JOINT_Z);
	 dBodySetPosition (robot_bodies[24],-LEG_X,-LEG_Y2,THIGH_JOINT_Z);



	double temp_tail_x = TAIL_X;

	//TAIL
	for (i=25; i<33; i++) {
    robot_bodies[i] = dBodyCreate (world);
    dMassSetCylinder (&m,1,1,TAIL_RADIUS, TAIL_LENGTH);
    dMassAdjust (&m,TAIL_MASS);
    dBodySetMass (robot_bodies[i],&m);
    robot_geoms[i] = dCreateCylinder(space,TAIL_RADIUS, TAIL_LENGTH);
    dGeomSetBody (robot_geoms[i],robot_bodies[i]);
	dBodySetPosition (robot_bodies[i],temp_tail_x, TAIL_Y, TAIL_Z);
	dMatrix3 R;
	dRFromAxisAndAngle (R,0,1,0,-1.57);
	dBodySetRotation (robot_bodies[i], R);

	temp_tail_x = temp_tail_x - TAIL_LENGTH;

  }
 	
  n_robot_parts = 32;
}


void initialize( )
{
  int j;
  dReal erp, cfm;

  // setup pointers to drawstuff callback functions
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
 
  dInitODE ();

  // create world
  world = dWorldCreate( );
  space = dSimpleSpaceCreate (0);
  dWorldSetGravity( world, 0.0, 0.0, -9.81 );
  dWorldSetCFM( world, 1e-7 );

  erp = dWorldGetERP( world );
  cfm = dWorldGetCFM( world );
 

  contact_joint_group = dJointGroupCreate( 0 );
  robot_joint_group = dJointGroupCreate( 0 );

  create_bodies_and_geoms( );

  plane = dCreatePlane(space, 0, 0, 1, 0);

  attach_joints();

  club_joint_ff = 0.0;


   Back_left_P1 = dBodyGetPosition (robot_bodies[1]);
 Back_right_P1 = dBodyGetPosition (robot_bodies[3]);
 Front_left_P1 = dBodyGetPosition (robot_bodies[0]);
 Front_right_P1 = dBodyGetPosition (robot_bodies[2]);
	dReal_to_vector(Back_left_P1, back_left_P1);
	dReal_to_vector(Back_right_P1, back_right_P1);
	dReal_to_vector(Front_left_P1, front_left_P1);
	dReal_to_vector(Front_right_P1, front_right_P1);

 const dReal* temp_height = dBodyGetPosition (robot_bodies[12]);
 front_frame_position = temp_height[0];
 initial_front_leg_frame_height = temp_height[2];
 prev_front_leg_frame_height = temp_height[2];

 temp_height = dBodyGetPosition (robot_bodies[17]);
 back_frame_position = temp_height[0];
 initial_back_leg_frame_height = temp_height[2];
 prev_back_leg_frame_height = temp_height[2];

 back_left_P2[0] = 0;
 back_left_P2[1] = 0;
 back_left_P2[2] = 0;
 back_right_P2[0] = 0;
 back_right_P2[1] = 0;
 back_right_P2[2] = 0;
 front_left_P2[0] = 0;
 front_left_P2[1] = 0;
 front_left_P2[2] = 0;
 front_right_P2[0] = 0;
 front_right_P2[1] = 0;
 front_right_P2[2] = 0;
 back_velocity = 0.0;
 front_velocity = 0.0;

	  get_P2(front_left_P1, dBodyGetPosition(robot_bodies[12]),front_left_P2,1 );
	get_P2(back_left_P1, dBodyGetPosition(robot_bodies[17]),back_left_P2,-1 );
	 get_P2(front_right_P1, dBodyGetPosition(robot_bodies[12]),front_right_P2,1 );
	get_P2(back_right_P1, dBodyGetPosition(robot_bodies[17]),back_right_P2,-1 );




  for ( j = 0; j < MAX_N_ROBOT_PARTS; j++ )
    {	
		target_angles[j] = 0.0;
      joint_positions[j] = 0.0;
      joint_velocities[j] = 0.0;
      joint_torques[j] = 0.0;
      joint_positions_desired[j] = 0.0;
	  k_pos[j] = 0.0;
      k_vel[j] = 0.0;
		//prev_target_angles[j] = 0.0;
    }
}

void clean_up()
{

  dJointGroupDestroy( contact_joint_group );
  dJointGroupDestroy( robot_joint_group );
  dSpaceDestroy( space );
  dWorldDestroy( world );
  dGeomDestroy( plane);
  dCloseODE();
}


int main( int argc, const char **argv )
{

  initialize();

  // run simulation
  dsSimulationLoop( argc, (char **) argv, WINDOW_WIDTH, WINDOW_HEIGHT, &fn );

  clean_up();
  return 0;
}