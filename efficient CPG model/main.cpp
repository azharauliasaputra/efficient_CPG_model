#include <stdio.h>
#include <stdlib.h>
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "CPG.h"
#include "locoGenerator.hpp"
//#include "projection.h"
//#include "gng.hpp"
//#include "NN.hpp"
//#include "SNN.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


#ifdef dDOUBLE
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawBox     dsDrawBoxD
#define dsDrawLine    dsDrawLineD
#define dsDrawTriangle    dsDrawTriangleD
#endif


double desireDistance;
void Pcontrol();

dWorldID      world;        // ìÆóÕäwåvéZópÇÃÉèÅ[ÉãÉh (for dynamics)
dSpaceID      space;        // è’ìÀåüèoópÇÃÉXÉyÅ[ÉX (for collision)
dGeomID       ground, footGround[3][4], contactObject[30], objectGround[num_obj * num_obj + 3], tladder[laddernum], tBuilding[3];       // ínñ  (ground)
dJointID     foot[4];       // ínñ  (ground)
dJointGroupID contactgroup; // ê⁄êGì_ÉOÉãÅ[Év (contact group for q)
dsFunctions   fn;           // ÉhÉçÅ[ÉXÉ^ÉbÉtÇÃï`âÊä÷êî (function of drawstuff)
dJointFeedback feedback[12];  // ÉtÉBÅ[ÉhÉoÉbÉNç\ë¢ëÃ
// start simulation - set viewpoint

//static  float   xyz[3] = {0.8317f,-2.9817f,2.000f};
//static  float   hpr[3] = {101.0000f,-27.5000f,0.0000f};
//float   xyz2[3] = {0.8317,-2.9817,2.0};
float xyzz[3] = {  0.0f,   0.0f, 0.5f};
float hprz[3] = { 0.0f, -90.0f, 0.0f};
static  float   xyz[3] = {0.2317f,-0.8817f,0.6500f};
//static  float   xyz[3] = {0.0317f,-0.7817f,0.6500f};
static  float   hpr[3] = {101.0000f,-25.5000f,0.0000f};
float xyzg[3] = {    0.6f, 0.4f, 0.5f};          // éãì_[m]
float hprg[3] = { -145.50f, -18.5f, 0.0f};          // éãê¸[Åã]
//static  float   hpr[3] = {101.0000f,-25.5000f,0.0000f};
float xyzx[3] = {  0.5f,   0.0f, 0.15f};
float hprx[3] = { -180.0f, -2.0f, 0.0f};

float xyzy[3] = {  0.0f,   -0.5f, 0.15f};
float hpry[3] = { 88.0f, -6.00f, 0.0f};
//float   xyz2[3] = {0.8317,-2.9817,2.0};
float xyz2[3] = {0.2317f,-0.8817f,0.6500f};


dReal  SX = 0, SY = 0, SZ = 0.52;           // ì∑ëÃèdêSÇÃèâä˙à íu(initial positon of COG)
double leg_angle[LEG_NUM][LINK_NUM];

static float xyzn[3],hprn[3];

bool crashing = 0;

typedef struct {
  dBodyID  body;
  dGeomID  geom;
  dJointID joint;
  dReal    m,r,x,y,z; // éøó (weight)ÅCîºåa(radius)ÅCà íu(positin:x,y,z)
} MyLink;


MyLink leg[LEG_NUM][LINK_NUM],torso,torso2, torso3, torso4, torso5, tail, object[num_obj], ladder[laddernum+2];

typedef struct {
    
    dBodyID body;
    dGeomID geom;
    dReal size[3];
    dReal weight;
    dReal position[3];
    
} MyObject;
dGeomID coob[100];
int conum = 0;

static const dReal box_size[3] = {0.008, 0.02, 0.1},box_weight = 0.0001;
static const dReal box_posi[3] = { -0.34, 0.0, SZ+0.04};

float *crossProduct(float *v1, float *v2){
    float *v = (float *) malloc(sizeof (float) * 3);
    v[0] = v1[1]*v2[2] - v2[1]*v1[2];
    v[1] = -(v1[0]*v2[2] - v2[0]*v1[2]);
    v[2] = v1[0]*v2[1] - v2[0]*v1[1];
    return v;
}
float *rotation_y(float a, float *pos){
    float *pos1 = (float *) malloc(sizeof (float) * 3);
    
    pos1[0] = pos[2] * sin(a) + pos[0] * cos(a);
    pos1[1] = pos[1];
    pos1[2] = pos[2] * cos(a) - pos[0] * sin(a);
    
    return pos1;
}
float *rotation_z(float a, float *pos){
    float *pos1 = (float *) malloc(sizeof (float) * 3);
    
    pos1[0] = pos[0] * cos(a) - pos[1] * sin(a);
    pos1[1] = pos[0] * sin(a) + pos[1] * cos(a);
    pos1[2] = pos[2];
    
    return pos1;
}
//#define yMin -0.60086
#define yMin 0.2
#define yMax 0.60086
#define xMin -0.41421
#define xMax 0.41421

#define yRes (yMax - yMin)/(yNUM-1)
#define xRes (xMax - xMin)/(xNUM-1)


double rand1() /* uniform random number  */
{
    return((double)(rand()%30001)/30000.0);
}

double THETA[LEG_NUM][LINK_NUM] = {{0},{0},{0},{0}}; // ñ⁄ïWäpìx(target angle)
dReal  gait[12][LEG_NUM][JT_NUM] ;          // ñ⁄ïWäpìx (target angle of gait)
dReal  l1 = 0.5*0.05, l2 = 165/SCALE, l3  = 172/SCALE, l4 = 0.045, l5 = 0.07;  // ÉäÉìÉNí∑ 0.25 (lenth of links)

dReal  lx = 0.5, ly= 0.26, lz = 0.05;         // body sides
dReal  lx2 = 0.1, ly2= 0.1, lz2 = 0.15;         // body2 sides
dReal  rb1 = 0.035, rl1 = 0.08;
dReal  rb3 = 0.08, rl3 = 0.02;
dReal  rb4 = 0.035, rl4 = 0.18;
dReal  rb2 = 0.1, rl2 = 0.44;
dReal  rb5 = 0.028, rl5 = 0.3;
dReal  r1 = 0.02, r2 = 0.03, r3 = 0.03, r4 = 0.015, r5 = 0.015 ;     // leg radius
dReal  cx1 = (lx-r1)/2, cy1 = (ly+l1)/2;     //  (temporal variable)
double  c_x[LEG_NUM][LINK_NUM] = {{ cx1, cx1, cx1, cx1, cx1},{-cx1,-cx1,-cx1,-cx1,-cx1}, // (center of joints)
                                    {-cx1,-cx1,-cx1,-cx1,-cx1},{ cx1, cx1, cx1, cx1, cx1}};
double  c_y[LEG_NUM][LINK_NUM] = {{ cy1, cy1, cy1, cy1+(r3+r4)/2, cy1+(r3+r4)/2},{ cy1, cy1, cy1, cy1+(r3+r4)/2, cy1+(r3+r4)/2}, // (center of joints)
                                {-cy1,-cy1,-cy1,-cy1-(r3+r4)/2,-cy1-(r3+r4)/2},{-cy1,-cy1,-cy1,-cy1-(r3+r4)/2,-cy1-(r3+r4)/2}};
double  c_z[LEG_NUM][LINK_NUM] =  {{0, 0, -l2, -l2-l3}, {0, 0, -l2, -l2-l3},
                                    {0, 0, -l2, -l2-l3},{0, 0, -l2, -l2-l3}};

int CT = 0;
void  makeRobot(){
    dMatrix3 Rb;
    
    dReal torso_m = 3.003;                    //   (weight of torso)
    dReal torso_m2 = 0.1;                //   (weight of torso)
    dReal torso_m3 = 8.6;                    //   (weight of torso)
    dReal torso_m4 = 4.001;                   //   (weight of torso)
    dReal torso_m5 = 0.005;                  //   (weight of torso)
    dReal tail_m = 0.01;                    //   (weight of torso)
    dReal  l1m = 0.005,l2m = 3.5, l3m = 3.5,l4m = 0.4, l5m = 0.5; //  (weight of links)

    dReal x[LEG_NUM][LINK_NUM] = {{ cx1, cx1, cx1, cx1-(l4/2), cx1-l4},{-cx1,-cx1,-cx1,-cx1-(l4/2),-cx1-l4}, // (center of joints)
                                  {-cx1,-cx1,-cx1,-cx1-(l4/2),-cx1-l4},{ cx1, cx1, cx1, cx1-(l4/2), cx1-l4}};
    dReal y[LEG_NUM][LINK_NUM] = {{ cy1, cy1, cy1, cy1+0*(r3+r4), cy1+0*(r3+r4)},{ cy1, cy1, cy1, cy1+0*(r3+r4), cy1+0*(r3+r4)}, // (center of joints)
                                  {-cy1,-cy1,-cy1,-cy1-0*(r3+r4),-cy1-0*(r3+r4)},{-cy1,-cy1,-cy1,-cy1-0*(r3+r4),-cy1-0*(r3+r4)}};
  double z[LEG_NUM][LINK_NUM] = {                                  //  íu(link position, zç¿ïW)
                                {c_z[0][0],(c_z[0][0]+c_z[0][2])/2,c_z[0][2]-l3/2,z[0][2]-l3/2,z[0][3]-l5/2},
                                {c_z[0][0],(c_z[0][0]+c_z[0][2])/2,c_z[0][2]-l3/2,z[0][2]-l3/2,z[0][3]-l5/2},
                                {c_z[0][0],(c_z[0][0]+c_z[0][2])/2,c_z[0][2]-l3/2,z[0][2]-l3/2,z[0][3]-l5/2},
                                {c_z[0][0],(c_z[0][0]+c_z[0][2])/2,c_z[0][2]-l3/2,z[0][2]-l3/2,z[0][3]-l5/2}};
  dReal r[LINK_NUM]          =  { r1, r2, r3, r4, r5}; //  (radius of links)
  dReal length[LINK_NUM]     =  { l1, l2, l3, l4, l5}; // (length of links)
    dReal weight[LINK_NUM]     =  {l1m,l2m,l3m,l4m,l5m}; //   (weight of links)
    dReal axis_x[LEG_NUM][LINK_NUM] = {{ 1, 0, 0, 0, 0},{ 1, 0, 0, 0, 0},{ 1, 0, 0, 0, 0},{ 1, 0, 0, 0, 0}};
    dReal axis_y[LEG_NUM][LINK_NUM] = {{ 0, 1, 1, 1, 1},{ 0, 1, 1, 1, 1},{ 0, 1, 1, 1, 1},{ 0, 1, 1, 1, 1}};
    dReal axis_z[LEG_NUM][LINK_NUM] = {{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0},{ 0, 0, 0, 0, 0}};

    // ì∑ëÃÇÃê∂ê¨ (crate a torso)
    dMass mass;
    CT = 0;
    
    torso.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetBoxTotal(&mass,torso_m, lx, ly, lz);
    dBodySetMass(torso.body,&mass);
    torso.geom = dCreateBox(space,lx, ly, lz);
    dGeomSetBody(torso.geom, torso.body);
    dBodySetPosition(torso.body, SX, SY, SZ);
    contactObject[CT] = torso.geom;
    CT++;
    
    torso2.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsule(&mass,torso_m2, 1, rb1, rl1);
    dMassAdjust(&mass,torso_m2);
    dBodySetMass(torso2.body,&mass);
    torso2.geom = dCreateCapsule(space,rb1,rl1);
    dBodySetPosition(torso2.body, -0.05+SX-lx/1.9, SY, SZ+lz);
    dGeomSetBody(torso2.geom, torso2.body);
    dRFromZAxis(Rb, 1, 0, -1);
    dBodySetRotation(torso2.body, Rb);
    contactObject[CT] = torso2.geom;
    CT++;
    
    torso3.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsule(&mass,torso_m3, 1, rb2, rl2);
    dMassAdjust(&mass,torso_m3);
    dBodySetMass(torso3.body,&mass);
    torso3.geom = dCreateCapsule(space,rb2,rl2);
    dBodySetPosition(torso3.body, SX, SY, SZ);
    dGeomSetBody(torso3.geom, torso3.body);
    dRFromZAxis(Rb, 1, 0, 0);
    dBodySetRotation(torso3.body, Rb);
    contactObject[CT] = torso3.geom;
    CT++;
    
    torso4.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsule(&mass,torso_m4, 1, rb3,rl3);
    dMassAdjust(&mass,torso_m4);
    dBodySetMass(torso4.body,&mass);
    torso4.geom = dCreateCapsule(space,rb3,rl3);
    dBodySetPosition(torso4.body, SX-lx/1.7, SY, SZ+lz);
    dGeomSetBody(torso4.geom, torso4.body);
    dRFromZAxis(Rb, 1, 0, 0.0);
    dBodySetRotation(torso4.body, Rb);
    contactObject[CT] = torso4.geom;
    CT++;
    
//    torso5.body  = dBodyCreate(world);
//    dMassSetZero(&mass);
//    dMassSetCapsule(&mass,torso_m5, 1, rb4,rl4);
//    dMassAdjust(&mass,torso_m5);
//    dBodySetMass(torso5.body,&mass);
//      torso5.geom = dCreateCapsule(space,rb4,rl4);
//    dBodySetPosition(torso5.body, -0.1+SX-lx/1.6, SY, SZ+lz);
//    dGeomSetBody(torso5.geom, torso5.body);
//    dRFromZAxis(Rb, 1, 0, 0.2);
//    dBodySetRotation(torso5.body, Rb);
    
    tail.body  = dBodyCreate(world);
    dMassSetZero(&mass);
    dMassSetCapsule(&mass,tail_m, 1, rb5,rl5);
    dMassAdjust(&mass,tail_m);
    dBodySetMass(tail.body,&mass);
    tail.geom = dCreateCapsule(space,rb5,rl5);
    dBodySetPosition(tail.body, SX+lx/1.9, SY, SZ);
    dGeomSetBody(tail.geom, tail.body);
    dRFromZAxis(Rb, 1, 0, -0.4);
    dBodySetRotation(tail.body, Rb);
    // ãrÇÃê∂ê¨ (create 4 legs)
    dMatrix3 R,R1,R2;                          // âÒì]çsóÒ
    dRFromAxisAndAngle(R,1,0,0,M_PI/2);  // 90ìxâÒì]Çµínñ Ç∆ïΩçs
    dRFromAxisAndAngle(R1,0,1,0,M_PI/2);  // 90ìxâÒì]Çµínñ Ç∆ïΩçs
    dRFromAxisAndAngle(R2,0,1,0,-M_PI/10);  // 90ìxâÒì]Çµínñ Ç∆ïΩçs
  for (int i = 0; i < LEG_NUM; i++) {
    for (int j = 0; j < LINK_NUM; j++) {
        leg[i][j].body = dBodyCreate(world);
        if (j == 0) dBodySetRotation(leg[i][j].body,R);
        if (j == 3) dBodySetRotation(leg[i][j].body,R1);
        if (j == 4) dBodySetRotation(leg[i][j].body,R2);
        dBodySetPosition(leg[i][j].body, SX+x[i][j], SY+y[i][j], SZ+z[i][j]);
        dMassSetZero(&mass);
        dMassSetCapsuleTotal(&mass,weight[j],3,r[j],length[j]);
        dBodySetMass(leg[i][j].body, &mass);
        leg[i][j].geom = dCreateCapsule(space,r[j],length[j]);
        dGeomSetBody(leg[i][j].geom,leg[i][j].body);
        
        if(i == 1 || i == 2){
            contactObject[CT] = leg[i][j].geom;
            CT++;
        }
        
        if(j >= 2){
            footGround[j-2][i] = leg[i][j].geom;
        }
    }
  }

  // (create links and attach legs to the torso)
    int l = 0;
  for (int i = 0; i < LEG_NUM; i++) {
    for (int j = 0; j < LINK_NUM; j++) {
      leg[i][j].joint = dJointCreateHinge(world, 0);
      if (j == 0)
        dJointAttach(leg[i][j].joint, torso.body, leg[i][j].body);
      else
        dJointAttach(leg[i][j].joint, leg[i][j-1].body, leg[i][j].body);
        dJointSetHingeAnchor(leg[i][j].joint, SX+c_x[i][j], SY+c_y[i][j],SZ+c_z[i][j]);
        dJointSetHingeAxis(leg[i][j].joint, axis_x[i][j], axis_y[i][j],axis_z[i][j]);
        if(j <= 2){
            dJointSetFeedback(leg[i][j].joint,&feedback[l]);
            l++;
        }
      
    }
  }
    torso2.joint=dJointCreateFixed(world,0);
    dJointAttach(torso2.joint,torso3.body,torso2.body);
    dJointSetFixed(torso2.joint);
    torso2.joint=dJointCreateFixed(world,0);
    dJointAttach(torso2.joint,torso.body,torso2.body);
    dJointSetFixed(torso2.joint);
    torso3.joint=dJointCreateFixed(world,0);
    dJointAttach(torso3.joint,torso.body,torso3.body);
    dJointSetFixed(torso3.joint);
    
    torso4.joint=dJointCreateHinge(world, 0);
    dJointAttach(torso4.joint, torso2.body, torso4.body);
    dJointSetHingeAnchor(torso4.joint, SX-lx/1.7, SY, SZ+lz);
    dJointSetHingeAxis(torso4.joint, 0, 1, 0);
    
    tail.joint=dJointCreateFixed(world,0);
    dJointAttach(tail.joint,torso.body,tail.body);
    dJointSetFixed(tail.joint);
    tail.joint=dJointCreateFixed(world,0);
    dJointAttach(tail.joint,torso3.body,tail.body);
    dJointSetFixed(tail.joint);
}
/*** ÉçÉ{ÉbÉgÇÃï`âÊ (draw a robot) ***/
void drawRobot(){
   dReal r,length;
   dVector3 sides;
    
    
    // ì∑ëÃÇÃï`âÊ
    dsSetColor(1.3,2.3,1.3);
    dGeomBoxGetLengths(torso.geom,sides);
    dsDrawBox(dBodyGetPosition(torso.body),
              dBodyGetRotation(torso.body),sides);
    dsSetColor(1.3,1.3,1.3);
    dsDrawCapsule(dBodyGetPosition(torso2.body),
                  dBodyGetRotation(torso2.body),rl1,rb1);
    dsSetColor(1.3,1.3,1.3);
    //dGeomCapsuleGetParams(torso3.geom,sides);
    dsDrawCapsule(dBodyGetPosition(torso3.body),
                  dBodyGetRotation(torso3.body),rl2,rb2);
    
    dsSetColor(1.3,1.3,1.3);
    dsDrawCapsule(dBodyGetPosition(torso4.body),
                  dBodyGetRotation(torso4.body),rl3,rb3);
//    dsDrawCapsule(dBodyGetPosition(torso5.body),
//                  dBodyGetRotation(torso5.body),rl4,rb4);
    dsDrawCapsule(dBodyGetPosition(tail.body),
                  dBodyGetRotation(tail.body),rl5,rb5);

   // ãrÇÃï`âÊ
   for (int i = 0; i < LEG_NUM; i++) {
       if(i == 0)
           dsSetColor(1,0.0,0.0);
       if(i == 1)
           dsSetColor(0,1,0.0);
       if(i == 2)
           dsSetColor(1,1,0.0);
       if(i == 3)
           dsSetColor(0,0.0,1);
       for (int j = 0; j < LINK_NUM; j++ ) {
           dGeomCapsuleGetParams(leg[i][j].geom, &r,&length);
           if (j== 0) dsDrawCapsule(dBodyGetPosition(leg[i][j].body),
                                  dBodyGetRotation(leg[i][j].body),0.5*length,1.2*r);
           else       dsDrawCapsule(dBodyGetPosition(leg[i][j].body),
                                  dBodyGetRotation(leg[i][j].body),length,r);
       }
   }
}
int touch[4]={0,0,0,0};
int forceTouch[4]={0,0,0,0};
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    
    static const int MAX_CONTACTS = 100;
    int i,j,k;
    
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected (b1,b2)) return;
    
    dContact contact[MAX_CONTACTS];
    int check = 0;
    int numc = dCollide(o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact));
    
    if (numc > 0) {
        int i,j,k;
        if(check == 0){
            for (i=0; i<numc; i++) {
                contact[i].surface.mode  =  dContactSoftCFM | dContactSoftERP;
                contact[i].surface.mu       = dInfinity;
                contact[i].surface.slip1    = 0.5;
                contact[i].surface.slip2    = 0.5;
                contact[i].surface.soft_cfm = 1e-5;
                contact[i].surface.soft_erp = 0.9;
                dJointID c = dJointCreateContact(world,contactgroup,&contact[i]);
                dJointAttach (c,dGeomGetBody(contact[i].geom.g1),
                              dGeomGetBody(contact[i].geom.g2));
            }
        }
    }
    
    
    if((o1==footGround[0][0] && o2==ground)||(o2==footGround[0][0] && o1==ground))touch[0]=1;
    if((o1==footGround[0][1] && o2==ground)||(o2==footGround[0][1] && o1==ground))touch[1]=1;
    if((o1==footGround[0][2] && o2==ground)||(o2==footGround[0][2] && o1==ground))touch[2]=1;
    if((o1==footGround[0][3] && o2==ground)||(o2==footGround[0][3] && o1==ground))touch[3]=1;
    
}

double fitEnergy = 0;
double Energy0 = 0;
double selisihFSR = 0;
dReal forcez[4];

dReal angle1, angle2, angle3;
int posture = 2;
void  inverseKinematics(dReal x, dReal y, dReal z,
                     dReal *ang1, dReal *ang2, dReal *ang3,int posture){
//    printf("%.3f\t%.3f\t%.3f\t%.3f\n",x,y,z,l2+l3);
//    printf(" Haiii \n");
    
    double Resultan1, Resultan2, sA, sB, sC;
    double length = sqrt(x*x+y*y+z*z);
    if (length > (l2 + l3)*0.98) {
        
        double nRatio = ((l2 + l3)*0.9)/length;
        x = x * nRatio;
        y = y * nRatio;
        z = z * nRatio;
        
        *ang1 = atan2(y,-z);
        
        Resultan1 = sqrt(pow(z, 2) + pow(y, 2));
        if(Resultan1 > (l2+l3))Resultan1 = l2+l3;
        Resultan2 = sqrt(pow(Resultan1, 2) + pow(x, 2));
        if(Resultan2 > (l2+l3))Resultan2 = l2+l3;
        
        sC = acos(((pow(l2, 2))+(pow(l3, 2))-(pow(Resultan2, 2)))/(2 * l2 * l3));
        sA = asin(-x/Resultan2);
        sB = asin((l3 * sin(sC))/Resultan2);
        
        *ang2 = ( sB - sA );
        *ang3 = -M_PI + sC;
    }else{
        y = -y;
        
        *ang1 = atan2(y,-z);
        
        Resultan1 = sqrt(pow(z, 2) + pow(y, 2));
        if(Resultan1 > (l2+l3))Resultan1 = l2+l3;
        Resultan2 = sqrt(pow(Resultan1, 2) + pow(x, 2));
        if(Resultan2 > (l2+l3))Resultan2 = l2+l3;
        
        sC = acos(((pow(l2, 2))+(pow(l3, 2))-(pow(Resultan2, 2)))/(2 * l2 * l3));
        sA = asin(-x/Resultan2);
        sB = asin((l3 * sin(sC))/Resultan2);
        
        *ang2 = ( sB - sA );
        *ang3 = -M_PI + sC;
    }
}

float xyz0[3] = {  0.5f,  -1.2f, 1.0f};  // éãì_[m] (View point)
float hpr0[3] = {180.0f, -30.0f, 0.0f};  // éãê¸[Åã] (View direction)

float xyz1[3] = {  1.2f,  0.0f, 1.0f};  // éãì_[m] (View point)
float hpr1[3] = {121.0f, -10.0f, 0.0f};  // éãê¸[Åã] (View direction)

int reset = 0, lastreset = 0;

double len_x[3] ={0,0,0} ,len_y[3]={0,0,0}, len[2], alfa=0;
double roll = 0, pitch = 0, angvelPitch = 0, angvelRoll;
double Acc = 0, dataVel[2], kec[50], ang[4], velx[2], Accl[2], pos[2];
int ModeRunning = 1;
bool lastFinishStep = 0;

double fitMoveDistance_pos[7];
double fitMoveSpeed_pos[7];
double fitPattern_pos[7];
double fitPhase_pos[7];
double fitTiming_pos[7];
void initialStart(){
    
    touch[0] = 0;
    touch[1] = 0;
    touch[2] = 0;
    touch[3] = 0;
    
    dSpaceCollide(space,0,&nearCallback); // è’ìÀåüèo (collision detection)
    dWorldStep(world, 0.01);              // ÉXÉeÉbÉvçXêV (step a simulation)
    dJointGroupEmpty(contactgroup);       // ê⁄êGì_ÉOÉãÅ[ÉvÇãÛ (empty jointgroup)
    
    
    static int a[4] = {0,0,0,0};
    for(int i = 0; i<4; i++){
        if(touch[i] == 0) a[i] -= 2;
        else if(touch[i] == 1) a[i] ++;
        
        if(touch[i] == 1 && a[i] > -10) a[i] = 1;
        
        if(a[i] < -30) a[i] = -30;
        else if(a[i] >= 1){
            a[i] = 1;
            forceTouch[i] = 1;
        }else{
            forceTouch[i] = 0;
        }
        forceTouch[i] = touch[i];
    }

    
    for(int i = 0; i<3; i++){
        xyz0[i] = data_pos[i] + xyz1[i];
    }
    
    reset = 0;
    
    countTrain ++;
    if(countTrain > MaxTime){
        reset = 1;
        fclose(OutputFile);
    }
    if((reset == 1) || crashing == 1){
        
        dSpaceDestroy(space);
        dWorldDestroy(world);
        dCloseODE();
        
        initTrajectory();
        dInitODE();
        setDrawStuff();
        world        = dWorldCreate();
        space        = dHashSpaceCreate(0);
        contactgroup = dJointGroupCreate(0);
        ground       = dCreatePlane(space,0,0,1,0);
        dWorldSetGravity(world, 0, 0, -9.8);
        dWorldSetCFM(world, 1e-3); // CFMÇÃê›íË (global CFM)
        dWorldSetERP(world, 0.9); // CFMÇÃê›íË (global CFM)
        //    dWorldSetERP(world, 0.9);  // ERPÇÃê›íË (global ERP)
        
        
        
        makeRobot();
        
        reset = 0;
        countTrain = 0;
        
        ModeRunning ++;
        if(ModeRunning > 30){
            fclose(OutputFile2);
            fclose(OutputFile);
        }
        
        neuralOscInit();
        for(int i=0; i<4; i++){
            for(int j=0; j<3; j++){
                THETA[i][j] = 0;
            }
        }
        
    }
}


double heading, headingTarget, lengthTarget, dHeading;
double robot_x, robot_y;
double startPos = 0, finishPos = 0;
int stepCount = 0, countFinish = 0;
float maxMove = 0;
float GRIP[4] = {-phi/2,-phi/2,-phi/2,-phi/2};
void walk()
{
    clock_t T;
    double time_taken=0;
    double theta0[JT_NUM] = {0,-0.1,-PI/1};
    static int t = 0, steps = 0;
    int interval = 30;
    
    headingRobotAndPosition();
  
    GetPositionObject();
    ModeLad_y[0] = 220;
    ModeLad_y[1] = 220;
    ModeLad_y[2] = 220;
    ModeLad_y[3] = 220;
    ModeLad_x[0] = -50;
    ModeLad_x[1] = -50;
    ModeLad_x[2] = 0;
    ModeLad_x[3] = 0;
    ModeLad_z[0] = 0;
    ModeLad_z[1] = 0;
    ModeLad_z[2] = 0;
    ModeLad_z[3] = 0;
    heightStep = 90;
    lengthStepX = 0;
    lengthStepY = 0;
    lengthStepZ = 0;
    
    if(countTrain > 0){
        Trajectory_walking();
        for (int leg_no = 0; leg_no < LEG_NUM; leg_no++) {
                inverseKinematics((-x_foot[leg_no] + ModeLad_x[leg_no] + Offset[0][leg_no])/SCALE,
                                  (z_foot[leg_no] - ModeLad_z[leg_no] + Offset[2][leg_no])/SCALE,
                                  (y_foot[leg_no] - ModeLad_y[leg_no] + Offset[1][leg_no])/SCALE,
                                  &angle1, &angle2, &angle3,posture);
             
            THETA[leg_no][0] = angle1;
            THETA[leg_no][1] = angle2;
            THETA[leg_no][2] = angle3;
            
        }
        
        THETA[0][3] = GRIP[2];
        THETA[1][3] = GRIP[0];
        THETA[2][3] = GRIP[1];
        THETA[3][3] = GRIP[3];
    
        Pcontrol();
    }
    
    initialStart();
    
}
FILE *fp = fopen("/Users/azhar/Library/Mobile Documents/com~apple~CloudDocs/Research/4leggedRobot programming/4-leggedSpeedControl-efficientControl/angle_data_new.txt","w");
void Pcontrol()
{
    dReal kp = 30, fMax = 200;
    
//    fprintf(fp,"%d\t",tTime);
//    fprintf(fp,"%d\t%.3f\t%.3f\t%.3f\t%.3f\t",stepCount,THETA[1][0],THETA[1][1],THETA[1][2],THETA[1][3]);
    for (int i = 0; i < LEG_NUM; i++) {
        for (int j = 0; j < LINK_NUM; j++) {
            dReal tmp = dJointGetHingeAngle(leg[i][j].joint);
            
//            fprintf(fp,"%.3f\t",leg_angle[i][j]);
            
            dReal diff = THETA[i][j] - tmp;
            dReal u = kp * diff;
            if(u > 50) u = 50;
            if(u < -50) u = -50;
            dJointSetHingeParam(leg[i][j].joint,  dParamVel, u);
            dJointSetHingeParam(leg[i][j].joint, dParamFMax, fMax);
//            if(i == 1){
                fprintf(fp,"%.3f\t",tmp - leg_angle[i][j]);
            leg_angle[i][j] = tmp;
//            }
        }
    }
    fprintf(fp,"\n");
    if (stepCount >= 16) {
        fclose(fp);
    }
}

double z_pos[4] = {-0.03,0.03,0.03,-0.03};


double fitMoveDistance = 0, fitMoveSpeed = 0;
double fitStepLength = 0;
double fitCountStep = 0;
double fitPhase = 0;
double fitStability = 0;
double fitPattern = 0;
double fitTiming = 0;


float data_pos[3];
void GetPositionObject(){
    int i;
    const dReal *pos0= dBodyGetPosition(torso.body);     // current position
    const dReal *rot0= dBodyGetRotation(torso.body);     // current posture
    const dReal *vel = dBodyGetLinearVel(torso.body);
    angvelRoll = (rot0[9] - roll)/0.005;
    angvelPitch = (rot0[8] - pitch)/0.005;
    roll = rot0[9]+0.00;
    pitch = rot0[8];
    
    fitStability += abs(angvelRoll) + abs(angvelPitch);
    for(i = 0; i<3; i++){
        data_pos[i] = pos0[i];
    }
    
    len_x[2] = len_x[1]; len_y[2] = len_y[1];
    len_x[1] = len_x[0]; len_y[1] = len_y[0];
    len_x[0] = pos0[0]; len_y[0] = pos0[1];
    
    velx[1] = velx[0];
    velx[0] = vel[0];// * cos(rot0[1] * PI/2) + vel[1] * sin(rot0[1] * PI/2);
    
    Accl[0] = (velx[0] - velx[1])/0.02;
    
    if(roll >= -0.4 && roll <= 0.4 && pitch >= -0.4 && pitch <= 0.4) reset = 0;
    else reset = 1;
    
    if(countTrain > 50){
        fitMoveDistance = sqrt((-data_pos[0])*(-data_pos[0]) + (data_pos[1])*(data_pos[1]));
//        fitMoveSpeed += (0 - (len_x[0] - len_x[1])) * (0  - (len_x[0] - len_x[1]));
        fitMoveSpeed += abs(Accl[0]);//(0 - (len_x[0] - len_x[1])) * (0  - (len_x[0] - len_x[1]));
    }
    
}
/*** ÉVÉ~ÉÖÉåÅ[ÉVÉáÉìÉãÅ[Év (Simulation Loop) ***/



bool testingMode = 0;
static int jval = 0;
static int maxjval = 0;
bool autoCam = 1;

const dReal *pb;

void simLoop(int pause)
{
    int h,i,j,k;
    static int flag = 1;
    dContactGeom c[4];
    double error;
    
    RungeKuttaGill_NeuOscMatsuoka(countTrain);
    walk();
    if(autoCam == 1){
        for(int i = 0; i<3; i++){
            xyz0[i] = pb[i] + xyz1[i];
        }
        dsSetViewpoint(xyz0,hpr0);
    }
    drawRobot();
}

/*** éãì_Ç∆éãê¸ÇÃê›íË (Set view point and direction) ***/
void start()
{
    int i;
    float xyz[3] = {  2.5f,  -1.2f, 2.0f};  // éãì_[m] (View point)
    float hpr[3] = {100.0f, -35.0f, 0.0f};  // éãê¸[Åã] (View direction)
    
                   // éãì_Ç∆éãê¸ÇÃê›íË (Set View point and direction)
    
    dsSetViewpoint(xyz,hpr);
    dsSetSphereQuality(3);
    dsSetCapsuleQuality(6);
}

static void command (int cmd)
{
    int i;
    switch (cmd) {
        case 'a': case 'A':
            printf("test\n");
            for(i = 0; i<3; i++){
                xyz0[i] = data_pos[i] + xyz1[i];
            }
            dsSetViewpoint(xyz0,hpr0);
            break;
        case '1': tStep[0] = 1;
            break;
        case '2': tStep[1] = 1;
            break;
        case '3': tStep[2] = 1;
            break;
        case '4': tStep[3] = 1;
            break;
        case 'i': stepLengthX = 80;
            stepLengthZ = 0;
            DirectionValue = 0;
            break;
        case 'k': stepLengthX = -80;
            stepLengthZ = 0;
            DirectionValue = 0;
            break;
        case 'j': stepLengthZ = -50;
            stepLengthX = 0;
            DirectionValue = 0;
            break;
        case 'l':
            stepLengthZ = 50;
            stepLengthX = 0;
            DirectionValue = 0;
            break;
        case 'o': DirectionValue = 0.08;
            stepLengthX = 0;
            stepLengthZ = 0;
            break;
        case 'u': DirectionValue = -0.08;
            stepLengthX = 0;
            stepLengthZ = 0;
            break;
        case 'c':
            if(autoCam) autoCam = false;
            else autoCam = true;
            break;
        case 'p':
            h_leg2[0] = 1;h_leg2[1] = 1;
            break;
        default:
            stepLengthX = 0;
            stepLengthZ = 0;
            DirectionValue = 0;
            break;
    }
}
/*** ÉhÉçÅ[ÉXÉ^ÉbÉtÇÃê›íË (Set drawstuff) ***/
void setDrawStuff() {
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command  = &command;
    fn.path_to_textures = "/Users/Azhar/ode-0.13/drawstuff/textures";
}
double rangeRotTarget = 45.0;
double rangeRotTarget2 = 5.0;
double rangeLenTarget = 0.02;
double rangeLenTarget2 = 2.0;

void headingRobotAndPosition()
{
    int i;
    const dReal *rot;
    
    pb  = dBodyGetPosition(torso.body);     // 台車重心の絶対座標
    rot = dBodyGetRotation(torso.body);     // 台車重心の絶対座標
    
    
    if(rot[1] > 0)      heading = atan(rot[0]/rot[1]) * rad;
    else if(rot[1] < 0) heading = 180 + atan(rot[0]/rot[1]) * rad;
    heading = -heading;// + 180;
    if(heading < -180)heading = heading + 360;
    robot_x = -pb[0];
    robot_y = pb[1];
    
    
}
void stop(){
    lengthStepX = 0;
}

double minVal(double a, double b){
    double x;
    if(a < b)
        x = a;
    else
        x = b;
    
    return x;
}
int countTrain = 0;
int main(int argc, char *argv[])
{
    int tt=0;
    
    ReadParamFile();
    OpenOutputFile();
    
    dInitODE();
    setDrawStuff();
    world        = dWorldCreate();
    space        = dHashSpaceCreate(0);
    contactgroup = dJointGroupCreate(0);
    ground       = dCreatePlane(space,0,0,1,0);
    dWorldSetGravity(world, 0, 0, -9.8);
    dWorldSetCFM(world, 1e-3); // CFMÇÃê›íË (global CFM)
    dWorldSetERP(world, 0.9); // CFMÇÃê›íË (global CFM)
    //    dWorldSetERP(world, 0.9);  // ERPÇÃê›íË (global ERP)
    makeRobot();
    
    neuralOscInit();
    
    ModeRunning = 0;
    jval = 0;
    initTrajectory();
    neuralOscInit();
    dsSimulationLoop(0,0,800,480,&fn);
    
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
#define sensorRange 30
float robotHeight = 0.6;
float groundHeight = 0.0;
void drawSensorCondition(struct gng *net){
    
}


