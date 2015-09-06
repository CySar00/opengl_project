/**
		Project Grafikwn 2013-2014 
*/

#define GL_GLEXT_PROTOTYPES

#include <GL/GLee.h>

#ifdef __APPLE__ 
#include <GLUT/glut.h> 
#else
#include <GL/glut.h> 
#endif 

#include <fstream>
#include <iostream> 
#include <cmath> 
#include <string> 
#include <set>
#include <cstdlib>
#include <vector>
#include <ctime>

#include <assert.h>
#include <stdlib.h> 
#include <math.h>
#include <memory.h> 

#include <GL/glext.h>
#include <GL/freeglut.h>
#include <GL/gl.h> 
#include <GL/glu.h>

#include <SOIL/SOIL.h> 

using namespace std; 

const float GRAVITY=10.0f;
const float QUAD_SIZE=2.0f;		//quad size 
const float TIME_BETWEEN_UPDATES=0.01;	//the amount of time that we handle collisions and the effects of gravity 
const int  TIMER_MS=25;			//the number of millseconds to which the timer is set 


/*	Vec3f class		*/ 

class Vec3f {
	private:
		float v[3];
	public:
		Vec3f();
		Vec3f(float x, float y, float z);
		
		float &operator[](int index);
		float operator[](int index) const;
		
		Vec3f operator*(float scale) const;
		Vec3f operator/(float scale) const;
		Vec3f operator+(const Vec3f &other) const;
		Vec3f operator-(const Vec3f &other) const;
		Vec3f operator-() const;
		
		const Vec3f &operator*=(float scale);
		const Vec3f &operator/=(float scale);
		const Vec3f &operator+=(const Vec3f &other);
		const Vec3f &operator-=(const Vec3f &other);
		
		float magnitude() const;
		float magnitudeSquared() const;
		Vec3f normalize() const;
		float dot(const Vec3f &other) const;
		Vec3f cross(const Vec3f &other) const;
};

Vec3f operator*(float scale, const Vec3f &v);
std::ostream &operator<<(std::ostream &output, const Vec3f &v);

Vec3f::Vec3f() {
	
}

Vec3f::Vec3f(float x, float y, float z) {
	v[0] = x;
	v[1] = y;
	v[2] = z;
}

float &Vec3f::operator[](int index) {
	return v[index];
}

float Vec3f::operator[](int index) const {
	return v[index];
}

Vec3f Vec3f::operator*(float scale) const {
	return Vec3f(v[0] * scale, v[1] * scale, v[2] * scale);
}

Vec3f Vec3f::operator/(float scale) const {
	return Vec3f(v[0] / scale, v[1] / scale, v[2] / scale);
}

Vec3f Vec3f::operator+(const Vec3f &other) const {
	return Vec3f(v[0] + other.v[0], v[1] + other.v[1], v[2] + other.v[2]);
}

Vec3f Vec3f::operator-(const Vec3f &other) const {
	return Vec3f(v[0] - other.v[0], v[1] - other.v[1], v[2] - other.v[2]);
}

Vec3f Vec3f::operator-() const {
	return Vec3f(-v[0], -v[1], -v[2]);
}

const Vec3f &Vec3f::operator*=(float scale) {
	v[0] *= scale;
	v[1] *= scale;
	v[2] *= scale;
	return *this;
}

const Vec3f &Vec3f::operator/=(float scale) {
	v[0] /= scale;
	v[1] /= scale;
	v[2] /= scale;
	return *this;
}

const Vec3f &Vec3f::operator+=(const Vec3f &other) {
	v[0] += other.v[0];
	v[1] += other.v[1];
	v[2] += other.v[2];
	return *this;
}

const Vec3f &Vec3f::operator-=(const Vec3f &other) {
	v[0] -= other.v[0];
	v[1] -= other.v[1];
	v[2] -= other.v[2];
	return *this;
}

float Vec3f::magnitude() const {
	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float Vec3f::magnitudeSquared() const {
	return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

Vec3f Vec3f::normalize() const {
	float m = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	return Vec3f(v[0] / m, v[1] / m, v[2] / m);
}

float Vec3f::dot(const Vec3f &other) const {
	return v[0] * other.v[0] + v[1] * other.v[1] + v[2] * other.v[2];
}

Vec3f Vec3f::cross(const Vec3f &other) const {
	return Vec3f(v[1] * other.v[2] - v[2] * other.v[1],
				 v[2] * other.v[0] - v[0] * other.v[2],
				 v[0] * other.v[1] - v[1] * other.v[0]);
}

Vec3f operator*(float scale, const Vec3f &v) {
	return v * scale;
}

ostream &operator<<(ostream &output, const Vec3f &v) {
	cout << '(' << v[0] << ", " << v[1] << ", " << v[2] << ')';
	return output;
}


 /*	Collisions 		*/

//returns a random float number from 0 to 1 
float randomFloat(){
	return (float)rand()/((float)RAND_MAX+1);
}

//Stores information regarding the ball:
struct Ball{
	Vec3f v;		//velocity 
	Vec3f pos; 		//position 
	float r;		//radius 
	Vec3f color; 
};

//wall walls 
enum Wall{WALL_LEFT,WALL_RIGHT,WALL_FAR,WALL_NEAR,WALL_TOP,WALL_BOTTOM}; 

//stores a pair of balls 
struct BallPair{
	Ball*ball1; 
	Ball*ball2; 
};

//stores a ball and wall pair 
struct BallWallPair{
	Ball*ball; 
	Wall wall; 
}; 


//Octree Code 
const int MAX_OCTREE_DEPTH=6; 
const int MIN_BALLS_PER_OCTREE=3; 
const int MAX_BALLS_PER_OCTREE=6; 
//Our data structure for making collision detection faster
class Octree {
	private:
		Vec3f corner1; //(minX, minY, minZ)
		Vec3f corner2; //(maxX, maxY, maxZ)
		Vec3f center;//((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2)
		
		/* The children of this, if this has any. 
		   children[0][*][*] are the children with x coordinates ranging from minX to centerX.
		   children[1][*][*] are the children with x coordinates ranging from centerX to maxX. 

		   children[*][0][*] are the children with y coordinates ranging from minY to centerY.
		   children[*][1][*] are the children with y coordinates ranging from centerY to maxY.  

		   
		   children[*][*][0] are the children with z coordinates ranging from minZ to centerZ.
		   children[*][*][1] are the children with z coordinates ranging from centerZ to maxZ.  
		 */

		Octree *children[2][2][2];
		//Whether this has children
		bool hasChildren;
		//The balls in this, if this doesn't have any children
		set<Ball*> balls;
		//The depth of this in the tree
		int depth;
		//The number of balls in this, including those stored in its children
		int numBalls;
		
		//Adds a ball to or removes one from the children of this
		void fileBall(Ball* ball, Vec3f pos, bool addBall) {
			//Figure out in which child(ren) the ball belongs
			for(int x = 0; x < 2; x++) {
				if (x == 0) {
					if (pos[0] - ball->r > center[0]) {
						continue;
					}
				}
				else if (pos[0] + ball->r < center[0]) {
					continue;
				}
				
				for(int y = 0; y < 2; y++) {
					if (y == 0) {
						if (pos[1] - ball->r > center[1]) {
							continue;
						}
					}
					else if (pos[1] + ball->r < center[1]) {
						continue;
					}
					
					for(int z = 0; z < 2; z++) {
						if (z == 0) {
							if (pos[2] - ball->r > center[2]) {
								continue;
							}
						}
						else if (pos[2] + ball->r < center[2]) {
							continue;
						}
						
						//Add or remove the ball
						if (addBall) {
							children[x][y][z]->add(ball);
						}
						else {
							children[x][y][z]->remove(ball, pos);
						}
					}
				}
			}
		}
		
		//Creates children of this, and moves the balls in this to the children
		void haveChildren() {
			for(int x = 0; x < 2; x++) {
				float minX;
				float maxX;
				if (x == 0) {
					minX = corner1[0];
					maxX = center[0];
				}
				else {
					minX = center[0];
					maxX = corner2[0];
				}
				
				for(int y = 0; y < 2; y++) {
					float minY;
					float maxY;
					if (y == 0) {
						minY = corner1[1];
						maxY = center[1];
					}
					else {
						minY = center[1];
						maxY = corner2[1];
					}
					
					for(int z = 0; z < 2; z++) {
						float minZ;
						float maxZ;
						if (z == 0) {
							minZ = corner1[2];
							maxZ = center[2];
						}
						else {
							minZ = center[2];
							maxZ = corner2[2];
						}
						
						children[x][y][z] = new Octree(Vec3f(minX, minY, minZ),
													   Vec3f(maxX, maxY, maxZ),
													   depth + 1);
					}
				}
			}
			
			//Remove all balls from "balls" and add them to the new children
			for(set<Ball*>::iterator it = balls.begin(); it != balls.end();
					it++) {
				Ball* ball = *it;
				fileBall(ball, ball->pos, true);
			}
			balls.clear();
			
			hasChildren = true;
		}
		
		//Adds all balls in this or one of its descendants to the specified set
		void collectBalls(set<Ball*> &bs) {
			if (hasChildren) {
				for(int x = 0; x < 2; x++) {
					for(int y = 0; y < 2; y++) {
						for(int z = 0; z < 2; z++) {
							children[x][y][z]->collectBalls(bs);
						}
					}
				}
			}
			else {
				for(set<Ball*>::iterator it = balls.begin(); it != balls.end();
						it++) {
					Ball* ball = *it;
					bs.insert(ball);
				}
			}
		}
		
		//Destroys the children of this, and moves all balls in its descendants
		//to the "balls" set
		void destroyChildren() {
			//Move all balls in descendants of this to the "balls" set
			collectBalls(balls);
			
			for(int x = 0; x < 2; x++) {
				for(int y = 0; y < 2; y++) {
					for(int z = 0; z < 2; z++) {
						delete children[x][y][z];
					}
				}
			}
			
			hasChildren = false;
		}
		
		//Removes the specified ball at the indicated position
		void remove(Ball* ball, Vec3f pos) {
			numBalls--;
			
			if (hasChildren && numBalls < MIN_BALLS_PER_OCTREE) {
				destroyChildren();
			}
			
			if (hasChildren) {
				fileBall(ball, pos, false);
			}
			else {
				balls.erase(ball);
			}
		}
		
		/* Helper fuction for potentialBallWallCollisions(vector).  Adds
		 * potential ball-wall collisions to cs, where w is the type of wall,
		 * coord is the relevant coordinate of the wall ('x', 'y', or 'z'), and
		 * dir is 0 if the wall is in the negative direction and 1 if it is in
		 * the positive direction.  Assumes that this is in the extreme
		 * direction of the coordinate, e.g. if w is WALL_TOP, the function
		 * assumes that this is in the far upward direction.
		 */
		void potentialBallWallCollisions(vector<BallWallPair> &cs,
										 Wall w, char coord, int dir) {
			if (hasChildren) {
				//Recursively call potentialBallWallCollisions on the correct
				//half of the children (e.g. if w is WALL_TOP, call it on
				//children above centerY)
				for(int dir2 = 0; dir2 < 2; dir2++) {
					for(int dir3 = 0; dir3 < 2; dir3++) {
						Octree *child;
						switch (coord) {
							case 'x':
								child = children[dir][dir2][dir3];
								break;
							case 'y':
								child = children[dir2][dir][dir3];
								break;
							case 'z':
								child = children[dir2][dir3][dir];
								break;
						}
						
						child->potentialBallWallCollisions(cs, w, coord, dir);
					}
				}
			}
			else {
				//Add (ball, w) for all balls in this
				for(set<Ball*>::iterator it = balls.begin(); it != balls.end();
						it++) {
					Ball* ball = *it;
					BallWallPair bwp;
					bwp.ball = ball;
					bwp.wall = w;
					cs.push_back(bwp);
				}
			}
		}
	public:
		//Constructs a new Octree.  c1 is (minX, minY, minZ), c2 is (maxX, maxY,
		//maxZ), and d is the depth, which starts at 1.
		Octree(Vec3f c1, Vec3f c2, int d) {
			corner1 = c1;
			corner2 = c2;
			center = (c1 + c2) / 2;
			depth = d;
			numBalls = 0;
			hasChildren = false;
		}
		
		//Destructor
		~Octree() {
			if (hasChildren) {
				destroyChildren();
			}
		}
		
		//Adds a ball to this
		void add(Ball* ball) {
			numBalls++;
			if (!hasChildren && depth < MAX_OCTREE_DEPTH &&
				numBalls > MAX_BALLS_PER_OCTREE) {
				haveChildren();
			}
			
			if (hasChildren) {
				fileBall(ball, ball->pos, true);
			}
			else {
				balls.insert(ball);
			}
		}
		
		//Removes a ball from this
		void remove(Ball* ball) {
			remove(ball, ball->pos);
		}
		
		//Changes the position of a ball in this from oldPos to ball->pos
		void ballMoved(Ball* ball, Vec3f oldPos) {
			remove(ball, oldPos);
			add(ball);
		}
		
		//Adds potential ball-ball collisions to the specified set
		void potentialBallBallCollisions(vector<BallPair> &collisions) {
			if (hasChildren) {
				for(int x = 0; x < 2; x++) {
					for(int y = 0; y < 2; y++) {
						for(int z = 0; z < 2; z++) {
							children[x][y][z]->
								potentialBallBallCollisions(collisions);
						}
					}
				}
			}
			else {
				//Add all pairs (ball1, ball2) from balls
				for(set<Ball*>::iterator it = balls.begin(); it != balls.end();
						it++) {
					Ball* ball1 = *it;
					for(set<Ball*>::iterator it2 = balls.begin();
							it2 != balls.end(); it2++) {
						Ball* ball2 = *it2;
						//This test makes sure that we only add each pair once
						if (ball1 < ball2) {
							BallPair bp;
							bp.ball1 = ball1;
							bp.ball2 = ball2;
							collisions.push_back(bp);
						}
					}
				}
			}
		}
		
		//Adds potential ball-wall collisions to the specified set
		void potentialBallWallCollisions(vector<BallWallPair> &collisions) {
			potentialBallWallCollisions(collisions, WALL_LEFT, 'x', 0);
			potentialBallWallCollisions(collisions, WALL_RIGHT, 'x', 1);
			potentialBallWallCollisions(collisions, WALL_BOTTOM, 'y', 0);
			potentialBallWallCollisions(collisions, WALL_TOP, 'y', 1);
			potentialBallWallCollisions(collisions, WALL_FAR, 'z', 0);
			potentialBallWallCollisions(collisions, WALL_NEAR, 'z', 1);
		}
};


//Basic Collision Mechanics  

//puts potential ball-ball collisions in potentialCollisions and it must return all actual collisions,but it doesnt neeed to return only actual collsions 
void potentialBallBallCollisions(vector<BallPair> &potentialCollisions,vector<Ball*>&balls,Octree*octree){
	
	octree->potentialBallBallCollisions(potentialCollisions); 

}

//puts potential ball-wall collisions in potentialCollisions,and it must return all actual collisions but it doesn't need to return only actual collisions: 
void potentialBallWallCollisions(vector<BallWallPair> &potentialCollisions,vector<Ball*> &balls, Octree* octree) {
    
    octree->potentialBallWallCollisions(potentialCollisions);
}


//moves all of the ball by v*dt 
void moveBalls(vector<Ball*> &balls,Octree*octree,float dt){
	for(unsigned int i=0;i<balls.size();i++){
		Ball*ball=balls[i];
		Vec3f oldPos=ball->pos;
		ball->pos+=ball->v*dt;
		octree->ballMoved(ball,oldPos);
	}
}

//decreases the y coordinate of v of each ball by GRAVITY*TIME_BETWEEN_UPDATES
void applyGravity(vector<Ball*>&balls){
	for(unsigned int i=0;i<balls.size();i++){
		Ball*ball=balls[i]; 
		ball->v-=Vec3f(0,GRAVITY*TIME_BETWEEN_UPDATES,0);
	}
	
}

//returns whether or not 2 balls are colliding 
bool testBallBallCollisions(Ball*b1,Ball*b2){
	//checking whether or not the balls are close enough 
	float r=b1->r+b2->r; 
	
	if((b1->pos-b2->pos).magnitudeSquared()<r*r){
		//checking whether or not the balls are moving toward echother 
		Vec3f netVelocity=b1->v-b2->v; 
		Vec3f displacement=b1->pos-b2->pos; 

		return netVelocity.dot(displacement)<0; 
	}else{
		return false ;
	}	
	
}

//Handles ball-ball collisions
void handleBallBallCollisions(vector<Ball*>&balls,Octree*octree){
	vector<BallPair>bps;
	potentialBallBallCollisions(bps,balls,octree);
	for(unsigned int i;i<bps.size();i++){
		BallPair bp=bps[i];
		Ball*b1=bp.ball1;
		Ball*b2=bp.ball2;
		if(testBallBallCollisions(b1,b2)){
			//makes the balls reflect off of eachother 
			Vec3f displacement=(b1->pos-b2->pos).normalize(); 
			b1->v-=2*displacement*b1->v.dot(displacement);
			b2->v-=2*displacement*b2->v.dot(displacement);
		}
	}
}


//returns the directions from the origin to the wall 
Vec3f wallDirection(Wall wall){
	switch(wall){
		case WALL_LEFT:
			return Vec3f(-1,0,0);
		case WALL_RIGHT:
			return Vec3f(1,0,0); 
		case WALL_FAR:
			return Vec3f(0,0,-1);
		case WALL_NEAR:
			return Vec3f(0,0,1); 
		case WALL_TOP:
			return Vec3f(0,1,0); 
		case WALL_BOTTOM:
			return Vec3f(0,-1,0); 
		default:
			return Vec3f(0,0,0); 
	}

}

//returns whether or not a ball and a wall are collidind 
bool testBallWallCollision(Ball*ball,Wall wall){
	Vec3f dir=wallDirection(wall); 
	//Checking whether or not the ball is far enough in the 'dir' directions and whether or not it's moviing towards the wall
 	return ball->pos.dot(dir)+ball->r>QUAD_SIZE/2 && ball->v.dot(dir)>0;

}

//handles every ball-wall collision
void handleBallWallCollisions(vector<Ball*>&balls,Octree*octree){
	vector<BallWallPair> bwps; 
	potentialBallWallCollisions(bwps,balls,octree);
	for(unsigned int i=0;i<bwps.size();i++){
		BallWallPair bwp=bwps[i];
		Ball*b=bwp.ball; 
		Wall w=bwp.wall; 
		
		if(testBallWallCollision(b,w)){
			//make the ball reflect off of the wall
			Vec3f dir=(wallDirection(w)).normalize(); 
			b->v-=2*dir*b->v.dot(dir); 

		}
	}
}

//Applies gravity and handles all collisions and should be called every 'TIME_BETWEEN_UPDATES' seconds 
void performUpdate(vector<Ball*>&balls,Octree*octree){
	applyGravity(balls); 
	handleBallBallCollisions(balls,octree); 
	handleBallWallCollisions(balls,octree); 
}


//advances the state of the balls by t timeUntilUpdate is the amount of time until the next call performance 
void advance(vector<Ball*>&balls,Octree*octree,float t,float &timeUntilUpdate){
	while(t>0){
		if(timeUntilUpdate<=t){
			moveBalls(balls,octree,timeUntilUpdate); 
			performUpdate(balls,octree); 
			t-=timeUntilUpdate; 
			timeUntilUpdate=TIME_BETWEEN_UPDATES;
	
		}else{
			moveBalls(balls,octree,t);
			timeUntilUpdate-=t;
			t=0;
		}
	}
}


/*                           Shaders                                 */ 
GLuint programObject; //a handler to the GLSL program used to update 
GLuint vertexShader;  // a handler to the vertex shader which is used internally
GLuint fragmentShader; //a handler to the fragment shader which is also used internally 

// vertex shader source 
static const char*vertexSource={
   "varying vec3 vVertex;"
   "varying vec3 vNormal;"
   "void main(){"
    " gl_Position=ftransform();"
    " vVertex=(gl_ModelViewMatrix*gl_Vertex).xyz;"
    " vNormal=gl_NormalMatrix*gl_Normal;"
   "}"
  
};

//fragment shader source:
static const char*fragmentSource={
    "varying vec3 vVertex;"
    "varying vec3 vNormal;"
    "uniform float amb;"
    "uniform vec3 color;"
    "float amb1=0.1;"
     "vec3 color1=vec3(1,0,0);"
     ""
     "void main(){"
     "    color=vec3(1,0,0);"
     "    amb=0.1;"
     "    vec3 N=normalize(vNormal);"
     "    vec3 V=normalize(vVertex);"
     "    vec3 R=reflect(V,N);"
     "    vec3 L=normalize(vec3(gl_LightSource[0].position));"
     "    vec3 ambient=color1*amb;"
     "    vec3 diffuse=color*(1.0-amb)*max(dot(L,N),0.0);"
     "    vec3 specular=vec3(1.0,1.0,1.0)*pow(max(dot(R,L),0.0),8.0);"
     "    gl_FragColor=vec4(ambient+diffuse+specular,1.0);"
     "    "
     "}"
};

//print out the information log for a shader object 
//@arg obj handles a program object 
static void printProgramInfo(GLuint obj){
  GLint infoLength=0,charsWritten=0; 
  glGetProgramiv(obj,GL_INFO_LOG_LENGTH,&infoLength);
  if(infoLength>2){
    GLchar*infoLog=new GLchar[infoLength];
    glGetProgramInfoLog(obj,infoLength,&charsWritten,infoLog);
    std::cerr<<infoLog<<std::endl; 
    delete infoLog;
  }

}

/*	Global Variables 			*/ 
char windowTitle[]="Project Grafikwn";
int windowHeight=600; 
int windowWidth=800;
//char *filename="image.bmp"; 

//Camera variables:
//Translate screen to x(left or right),y(up or down),z(zoom in or out) directions 
GLfloat X=0.0f;GLfloat Y=0.0f;GLfloat Z=-3.0f; 
//Rotate screen on x,y,z axis
//Rotate Screen on x,y,z axis
GLfloat rotX=0.0f;GLfloat rotY=0.0f;GLfloat rotZ=0.0f;
//Translate screen by using the gluLookAt function(left or right,up or down,zoom in or out)
GLfloat rotLx=0.0f;GLfloat rotLy=0.0f;GLfloat rotLz=0.0f;

GLUquadricObj *quad;
vector <Ball*>_balls; 	//all of the balls in play 
Octree* _octree;	//the octree with all of the balls 
float _timeUntilUpdate=0; //the amount of time untl performUpdate should be called 

GLuint texture[1];

//                   Light Components:

//Light Colors:
GLfloat diffuseLight[]={0.8f,0.8f,0.8f,1.0f};
GLfloat ambientDiffuseLight[]={0.5f,0.5f,0.5f,1.0f};
GLfloat ambientLight[]={0.1f,0.1f,0.1f,1.0f};
GLfloat specularLight[]={1.0f,1.0f,1.0f,1.0f};

enum lightColors{ambientColor=1,diffuseColor,specularColor};
int lcc=0; 

//Light Positions:
GLfloat centerLight[]={1.0f,1.0f,1.0f,1.0f};
GLfloat rightLight[]={2.0f,2.0f,0.0f,1.0f}; 
GLfloat topLeftLight[]={-2.0f,2.0f,2.0f,1.0f}; 

enum lightPostiotions{center=1,right1,top_left}; 
int lpp=0; 


//Light Types(directional,positional) Components: 

GLfloat positionalLightDirection[]={1.0f,1.0f,1.0f,1.0f};
GLfloat directionalLightDirection[]={10.0f,10.0f,-2.0f,1.0f};

enum lightTypes{directionalLight=1,positionalLight,ambientLightOnly,spotLight}; 
int ltt=0; 

//Boolean Variable for keyboard input(light,shader, and texture)
bool light;    //light on/off 
bool lp;       //'l' pressed? 

bool text;   //texture on/off 
bool tp;    //'T' pressed?

bool shader; //shader on/off

//deletes everything and is called when exiting the program 
void cleanup(){
	for(unsigned int i=0;i<_balls.size();i++){
		delete _balls[i];
	}
	delete _octree;
}

GLuint loadTexture(){
  
  texture[0]=SOIL_load_OGL_texture("image.bmp", 
				   SOIL_LOAD_AUTO,
				   SOIL_CREATE_NEW_ID,
				   SOIL_FLAG_NTSC_SAFE_RGB|SOIL_FLAG_MULTIPLY_ALPHA);

  if(texture[0]==0){return false;}
  
  //typical texture generation using data from the bitmap 
  glBindTexture(GL_TEXTURE_2D,texture[0]);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
  return true;

}

/* Initialize OpenGL Graphics */

void initGL() {
   glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
   glClearDepth(1.0f);                   // Set background depth to farthest
   glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
   glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
   glShadeModel(GL_SMOOTH);   // Enable smooth shading
   glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections

   programObject=glCreateProgram(); //creates a program object
   vertexShader=glCreateShader(GL_VERTEX_SHADER); //creates a vertex shader 
   fragmentShader=glCreateShader(GL_FRAGMENT_SHADER); //creates a fragment shader 
   printProgramInfo(programObject); 

   glShaderSource(vertexShader,1,&vertexSource,NULL); //assigns the vertex source 
   glShaderSource(fragmentShader,1,&fragmentSource,NULL); //assigns the fragmentsource 
   printProgramInfo(programObject); 

   //compiles and attaches the vertex shader onto the program 
   glCompileShader(vertexShader);
   glAttachShader(programObject,vertexShader); 
   printProgramInfo(programObject); 

   //compiles and attaches the fragment shader onto the program
   glCompileShader(fragmentShader); 
   glAttachShader(programObject,fragmentShader); 
   printProgramInfo(programObject); 

   //link shaders into a complete GLSL program 
   glLinkProgram(programObject);
   printProgramInfo(programObject); 
   
   //some extra code for checking if the initialization is ok 
   GLint progLinkSuccess; 
   glGetObjectParameterivARB(programObject,GL_OBJECT_LINK_STATUS_ARB,&progLinkSuccess); 
   
   glEnable(GL_NORMALIZE);
   glEnable(GL_COLOR_MATERIAL);
   glMaterialf(GL_FRONT,GL_SHININESS,50.0f);
   quad=gluNewQuadric();
   loadTexture();
 
}

/* Handler for window-repaint event. Called back when the window first appears and
   whenever the window needs to be re-painted. */
void display() {
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
   glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
   glLoadIdentity();                 // Reset the model-view matrix


   glPushMatrix(); 	// It is important to push the Matrix before calling glRotatef and glTranslatef
   glRotatef(rotX,1.0,0.0,0.0); // Rotate on x
   glRotatef(rotY,0.0,1.0,0.0); // Rotate on y
   glRotatef(rotZ,0.0,0.0,1.0); // Rotate on z
   glTranslatef(X,Y,Z);  // Move right and into the screen
   
   glBegin(GL_QUADS);   //begin drawing the 5 quad 3d scene 
   
   //right face(x=QUAD_SIZE/2);
   glColor3f(0.6f,0.0f,1.0f);
   glNormal3f(1.0f,0.0f,0.0f);
   glVertex3f(QUAD_SIZE,-QUAD_SIZE,-QUAD_SIZE);
   glVertex3f(QUAD_SIZE, QUAD_SIZE,-QUAD_SIZE);
   glVertex3f(QUAD_SIZE,QUAD_SIZE,QUAD_SIZE);
   glVertex3f(QUAD_SIZE,-QUAD_SIZE,QUAD_SIZE);

   //left face(x=-QUAD_SIZE/2);
   glColor3f(0.5f,0.0f,0.5f);
   glNormal3f(-1.0f,0.0f,0.0f);
   glVertex3f(-QUAD_SIZE,-QUAD_SIZE,-QUAD_SIZE);
   glVertex3f(-QUAD_SIZE,QUAD_SIZE,-QUAD_SIZE);
   glVertex3f(-QUAD_SIZE,QUAD_SIZE,QUAD_SIZE);
   glVertex3f(-QUAD_SIZE,-QUAD_SIZE,QUAD_SIZE);

   //top face(y=QUAD_SIZE/2)
   glColor3f(0.5f,0.0f,1.0f);
   glNormal3f(0.0f,1.0f,0.0f); 
   glVertex3f(QUAD_SIZE,QUAD_SIZE,QUAD_SIZE);
   glVertex3f(QUAD_SIZE,QUAD_SIZE,-QUAD_SIZE);
   glVertex3f(-QUAD_SIZE,QUAD_SIZE,-QUAD_SIZE);
   glVertex3f(-QUAD_SIZE,QUAD_SIZE,QUAD_SIZE);

   //bottom face(y=-QUAD_SIZE)
   glColor3f(0.6f,0.0f,0.7f);
   glNormal3f(0.0f,-1.0f,0.0f);
   glVertex3f(QUAD_SIZE,-QUAD_SIZE,-QUAD_SIZE);
   glVertex3f(QUAD_SIZE,-QUAD_SIZE,QUAD_SIZE);
   glVertex3f(-QUAD_SIZE,-QUAD_SIZE,QUAD_SIZE);
   glVertex3f(-QUAD_SIZE,-QUAD_SIZE,-QUAD_SIZE);
   
   //back face(z=-QUAD_SIZE/2)
   glColor3f(0.0f,0.3f,1.0f);
   glNormal3f(0.0f,0.0f,0.0f);
   glVertex3f(-QUAD_SIZE,-QUAD_SIZE,-QUAD_SIZE);
   glVertex3f(-QUAD_SIZE,QUAD_SIZE,-QUAD_SIZE);
   glVertex3f(QUAD_SIZE,QUAD_SIZE,-QUAD_SIZE);
   glVertex3f(QUAD_SIZE,-QUAD_SIZE,-QUAD_SIZE);
   
   
     glEnd();
         
     glShadeModel(GL_FLAT);
      //drawing one sphere
    glPushMatrix();
    //   glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,texture[0]);
   // glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
     gluQuadricTexture(quad,1);
     gluSphere(quad,0.05,30,30);
   // createSphere(0.05,30,30,30);
   // glutSolidSphere(0.05,30,30);
    glPopMatrix();
    
	for(int i=0;i<_balls.size();i++){
		Ball*ball=_balls[i]; 
		glPushMatrix(); 
		glTranslatef (ball->pos[0],ball->pos[1],ball->pos[2]);
		glColor3f(ball->color[0],ball->color[1],ball->color[2]); 
		gluQuadricTexture(quad,1); 
		gluSphere(quad,ball->r,30,30); 
		
		glPopMatrix();
	}

	glShadeModel(GL_SMOOTH);

    glFlush();

   glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
}

/* Handler for window re-size event. Called back when the window first appears and
   whenever the window is re-sized with its new width and height */

void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
   // Compute aspect ratio of the new window
   if (height == 0) height = 1;                // To prevent divide by 0
   GLfloat aspect = (GLfloat)width / (GLfloat)height;

   // Set the viewport to cover the new window
   glViewport(0, 0, width, height);

   // Set the aspect ratio of the clipping volume to match the viewport
   glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
   glLoadIdentity();             // Reset
   // Enable perspective projection with fovy, aspect, zNear and zFar
    gluPerspective(65.0f, aspect, 0.1f, 100.0f);
}


void changeLightTypes(GLenum which,int value){
  switch(value){
  case directionalLight:
    glLightfv(which,GL_DIFFUSE,(const GLfloat[]){0.8f,0.0f,0.0f,1.0f});
    glMaterialfv(GL_BACK,GL_DIFFUSE,(const GLfloat[]){1.0f,0.6f,0.0f});
    glLightfv(which,GL_POSITION,directionalLightDirection);
    break; 
  case positionalLight:
    glLightfv(which,GL_AMBIENT_AND_DIFFUSE,(const GLfloat[]){0.65f,0.0f,0.0f,1.0f});
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,(const GLfloat[]){1.0f,0.4f,0.0f});
    glLightfv(which,GL_POSITION,positionalLightDirection);
    break; 
  case ambientLightOnly:
    glLightfv(which,GL_AMBIENT,(const GLfloat[]){0.2,0.0f,0.0f,1.0f});
    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,(const GLfloat[]){1.0f,0.2f,0.0f});
    glLightfv(which,GL_POSITION,(const GLfloat[]){-2.0f,1.0f,-2.0f,1.0f});
    break;
  case spotLight:
    glLightfv(which,GL_SPECULAR,(const GLfloat[]){1.0f,1.0f,1.0f,1.0f});
    glLightfv(which,GL_POSITION,(const GLfloat[]){-2.0f,1.0f,2.0f,1.0f}); 
    glMaterialfv(GL_FRONT,GL_SPECULAR,(const GLfloat[]){0.5f,1.0f,0.5,1.0f}); 
    glMaterialf(GL_FRONT,GL_SHININESS,100.0f);
    //spotlight attributes 
    glLightf(which,GL_SPOT_CUTOFF,95.0f);
    glLightf(which,GL_SPOT_EXPONENT,2.0f);
    glLightfv(which,GL_SPOT_DIRECTION,(const GLfloat[]){2.0f,2.0f,-1.0f,0.0f});
  }
  glutPostRedisplay();
}


void changeLightColor(GLenum which,int value){
  switch(value){
  case ambientColor:
    glLightfv(which,GL_AMBIENT,ambientLight);
    glMaterialfv(GL_FRONT,GL_AMBIENT,(const  GLfloat[]){1.0f,0.2f,0.0});
    break; 
  case diffuseColor:
    glLightfv(which,GL_DIFFUSE,diffuseLight);
    glMaterialfv(GL_FRONT,GL_DIFFUSE, (const GLfloat[]){1.0f,0.6f,0.0f});
    break; 
  case specularColor:
    glLightfv(which,GL_SPECULAR,specularLight);
    glMaterialfv(GL_FRONT,GL_SPECULAR, (const GLfloat[]){1.0f,1.0f,0.0f});
    
    break;

  }
  glutPostRedisplay();
}

void changeLightPositions(GLenum which,int value){
  switch(value){
  case center:
    glLightfv(which,GL_POSITION,centerLight);
    break; 
  case right1:
    glLightfv(which,GL_POSITION,rightLight);
    break; 
  case top_left:
    glLightfv(which,GL_POSITION,topLeftLight); 
    break; 
  }
  glutPostRedisplay();
}

//this function is for the navigation and function keys
void keyboardButtons(unsigned char key,int x,int y){
switch(key){
    //rotates on xx' axis:
    case 'S':
       rotY+=0.5f;
        break;
    case 'F':
       rotY-=0.5f;
       break;
   //rotates on yy' axis
   case 'E':
      rotX+=0.5f;
      break;
   case 'D':
      rotX-=0.5f;
      break;
    //zooms in or out
   rotZ+=0.5f;
   break; 
  case 'B':
    Z+=0.5f;
     break;
  case 'V':
      Z-=0.5f;
      if(Z<-4.0f){
	Z=0.0f; 
    }
     
    break;
 case 's':
    shader=!shader;
    if(shader){
      glUseProgram(programObject);
    }else{
      glUseProgram(0);
    }
    
    break; 
 case 'T':
   tp=!tp;
   if(tp)
      glEnable(GL_TEXTURE_2D);
    else 
      glDisable(GL_TEXTURE_2D);

    break;

  case 'l':
    if(!lp)
      lp=true; 
      light=!light;       
      if(light){
	glEnable(GL_LIGHT0); 
        glEnable(GL_LIGHTING);
        
        glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER,0);
        glLightf(GL_LIGHT0,GL_CONSTANT_ATTENUATION,0.5f);
        glLightf(GL_LIGHT0,GL_LINEAR_ATTENUATION,-1.0f);
        glLightf(GL_LIGHT0,GL_QUADRATIC_ATTENUATION,0.8f);
    
      }else{
	glDisable(GL_LIGHT0);
        glDisable(GL_LIGHTING);
      }
    break;

  case 'L':
     ltt+=1; 
     changeLightTypes(GL_LIGHT0,ltt);
     if(ltt>3){
       ltt=0;
     }
    break;

  case 'C':
    lcc+=1; 
    changeLightColor(GL_LIGHT0,lcc);
    if(lcc>2){
      lcc=0;
    }
  case 'P':
     lpp+=1; 
     changeLightPositions(GL_LIGHT0,lpp);
     if(lpp>2){
       lpp=0;
     }    
     break;
		
case '+':
	for(int i=0;i<5;i++){
		Ball*ball=new Ball();
		ball->pos=Vec3f(8*randomFloat()-4,8*randomFloat()-4,8*randomFloat()-4); 
		ball->v=Vec3f(8*randomFloat()-4,8*randomFloat()-4,8*randomFloat()-4); 
		ball->r=0.05f; 
		ball->color=Vec3f(0.6f*randomFloat()+0.2f,0.6f*randomFloat()+0.2f,0.6f*randomFloat()+0.2f);
		_balls.push_back(ball);
		_octree->add(ball);

	}
    break;

case '-':
  
  if(_balls.size()>1){
    for(int i=_balls.size()-1;i>=0;i--){
    // delete _balls[i];
    // Ball*ball=_balls[i]; 
    
      _balls.pop_back();
    _octree->remove(_balls[i]);
	}  
  }
    break;
		



//exit app button
case 'q':
    cleanup();
    exit(1);

}
glutPostRedisplay();
}

//this function is called every TIMER_MS ms
void update(int value){
  advance(_balls,_octree,(float)TIMER_MS/1000.0f,_timeUntilUpdate); 
  glutPostRedisplay();
  glutTimerFunc(TIMER_MS,update,0);
}



/* Main function: GLUT runs as a console application starting at main() */
int main(int argc, char** argv) {
  srand((unsigned int)time(0));    //Seed the random number generator



   glutInit(&argc, argv);            // Initialize GLUT

	_octree=new Octree(Vec3f(-QUAD_SIZE/2,-QUAD_SIZE/2,-QUAD_SIZE),Vec3f(QUAD_SIZE/2,QUAD_SIZE/2,QUAD_SIZE/2),1);
   glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH); // Enable double buffered mode
   glutInitWindowSize(windowWidth,windowHeight);   // Set the window's initial width & height
   glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
   glutCreateWindow(windowTitle);          // Create window with the given title
   glutDisplayFunc(display);       // Register callback handler for window re-paint event
   glutReshapeFunc(reshape);       // Register callback handler for window re-size event
   glutKeyboardFunc(keyboardButtons);
	glutTimerFunc(TIMER_MS,update,0);

   initGL();                       // Our own OpenGL initialization

   glutMainLoop();                 // Enter the infinite event-processing loop
   return 0;
}



