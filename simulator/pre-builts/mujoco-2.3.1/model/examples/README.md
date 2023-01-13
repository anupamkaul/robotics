1. hello.xml

A simple model in MuJoCo’s MJCF format. 
It defines a plane fixed to the world, a light to better illuminate objects and cast shadows, 
and a floating box with 6 DOFs (this is what the “free” joint does).

If this model is simulated, the box will fall on the ground. Basic simulation code for the passive dynamics, without rendering, is given below.

#include "mujoco.h"
#include "stdio.h"

char error[1000];
mjModel* m;
mjData* d;

int main(void)
{
   // load model from file and check for errors
   m = mj_loadXML("hello.xml", NULL, error, 1000);
   if( !m )
   {
      printf("%s\n", error);
      return 1;
   }

   // make data corresponding to model
   d = mj_makeData(m);

   // run simulation for 10 seconds
   while( d->time<10 )
      mj_step(m, d);

   // free model and data
   mj_deleteData(d);
   mj_deleteModel(m);

   return 0;
}

----


2. example.xml

This model is a 7 degree-of-freedom arm “holding” a string with a cylinder attached at the other end. 
The string is implemented as a tendon with length limits. 
There is ball joint at the shoulder and pairs of hinge joints at the elbow and wrist. 
The box inside the cylinder indicates a free “joint”. 
The outer body element in the XML is the required worldbody. 
Note that using multiple joints between two bodies does not require creating dummy bodies.

The MJCF file contains the minimum information needed to specify the model. 
Capsules are defined by line-segments in space – in which case only the radius of the capsule is needed. 
The positions and orientations of body frames are inferred from the geoms belonging to them. 
Inertial properties are inferred from the geom shape under a uniform density assumption. 
The two sites are named because the tendon definition needs to reference them, but nothing else is named. 
Joint axes are defined only for the hinge joints but not the ball joint. 
Collision rules are defined automatically. Friction properties, gravity, simulation time step etc. are set to their defaults. 
The default geom color specified at the top applies to all geoms.

----



