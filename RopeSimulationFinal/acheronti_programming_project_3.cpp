#include <math.h>
#include <stdio.h>
#include <time.h>
#include <GL/glut.h>
#include <iostream>
#include <vector>

// SIMULATION SETTINGS
#define NUM_PARTICLES 50
#define SPRING_K 650
#define SPRING_D 0.045
#define GRAVITY (-9.81)
double TIME_STEPSIZE = 0.05;
bool PAUSED = true;

// ROPE SETTINGS
#define ROPE_WORLDSPACE_LENGTH 15
#define ROPE_WIDTH 2.0
#define ROPE_HEIGHT 4.0

// BALL SETTINGS
#define BALL_RADIUS 1.0
#define BALL_X (-2.0)
#define BALL_Y 0.0
#define BALL_Z 0.0

// PARTICLE SETTINGS
#define PARTICLE_R 1.0
#define PARTICLE_G 0.0
#define PARTICLE_B 0.0
#define PARTICLE_RADIUS 0.1
#define PARTICLE_MASS 1.0

  /////////////////////////////
 //		Vector 3 Class		//
/////////////////////////////
//	Implemented with double
//	Includes Magnitude, Dot and Cross product, Normalization

class Vec3 {
public:
	// Member variables
	double x, y, z;

	// Create a zero vector
	Vec3() { x = 0.0; y = 0.0; z = 0.0; }

	// Create new vector with values i (x), j (y), k (z)
	Vec3(double i, double j, double k) {
		x = i;
		y = j;
		z = k;
	}

	// return the magnitude of this vector
	double Magnitude() {
		double sum = (x * x) + (y * y) + (z * z);
		return sqrt(sum);
	}

	// Return the unit vector of this vector
	Vec3 Normalized() {
		double magnitude = Magnitude();
		Vec3 unit_vector = Vec3(x / magnitude, y / magnitude, z / magnitude);
		return unit_vector;
	}

	// Normalize without returning a vector
	void Normalize() {
		double magnitude = Magnitude();
		x = x / magnitude;
		y = y / magnitude;
		z = z / magnitude;
	}

	double Distance(const Vec3& pos) {
		return Vec3(pos.x - x, pos.y - y, pos.z - z).Magnitude();
	}

	// return the dot product of this vector dot other vector ( u * v )
	double DotProduct(const Vec3& other) {
		return (this->x * other.x) + (this->y * other.y) + (this->z * other.z);
	}

	// return the cross product of this vector dot other vector (u X v)
	Vec3 CrossProduct(const Vec3& other) {
		double i = (this->y * other.z) - (this->z * other.y);
		double j = (this->z * other.x) - (this->x * other.z);
		double k = (this->x * other.y) - (this->y * other.x);
		return Vec3(i, j, k);
	}

	// Add a vector
	Vec3 operator+(const Vec3& other) {
		return Vec3(x + other.x, y + other.y, z + other.z);
	}
	// Subtract a vector
	Vec3 operator-(const Vec3& other) {
		return Vec3(x - other.x, y - other.y, z - other.z);
	}
	// Return the negative
	Vec3 operator-() {
		return Vec3(0.0 - x, 0.0 - y, 0.0 - z);
	}
	// Scale a vector by a value
	Vec3 operator*(double scalar) {
		return Vec3(x * scalar, y * scalar, z * scalar);
	}
	// Divide by double divisor
	Vec3 operator/(double divisor) {
		return Vec3(x / divisor, y / divisor, z / divisor);
	}

	// Add a vector (modify values)
	void operator+=(const Vec3& other) {
		x += other.x; y += other.y; z += other.z;
	}
	// Subtract a vector (modify values)
	void operator-=(const Vec3& other) {
		x -= other.x; y -= other.y; z -= other.z;
	}
};

// Global Variables for Particle Integration
Vec3 Ball_Position{ BALL_X, BALL_Y, BALL_Z };
Vec3 FORCE_GRAVITY{ 0.0, GRAVITY, 0.0 };

  /////////////////////////////
 //		Particle Class		//
/////////////////////////////
//	Utilizes Verlet Integration
//	Given the sum of the forces acting on the particle
//	Compute displacement using previous and current position and acceleration of forces

class Particle {
public:
	Vec3 Position;
	Vec3 OldPosition;
	Vec3 Velocity;
	Vec3 Forces;

	double Mass;
	bool Anchored;

	Particle(Vec3 pos, double mass, bool anchored) {
		Velocity = Vec3(0.0, 0.0, 0.0);
		Forces = Vec3(0.0, 0.0, 0.0);
		Position = pos;
		OldPosition = pos;
		Mass = mass;
		Anchored = false;
	}

	Particle() { Particle({ 0.0,0.0,0.0 }, 0.0, false); }

	// Verlet integrate this particles displacement
	void Solve() {
		if (!Anchored) {
			Vec3 temp = Position; // consider using temp var

			Forces += FORCE_GRAVITY;

			Vec3 acceleration = (Forces / Mass);

			// VERLET INTEGRATION : CALCULATE VELOCITY USING ONLY OLD AND CURRENT POSITION
			// Lecture Slides Cloth Mistakes
			Velocity = (Position - OldPosition) * (1 - SPRING_D) + acceleration * TIME_STEPSIZE * TIME_STEPSIZE;
			Vec3 NewPosition = Position + Velocity;

			// Do not move the particle yet

			// Check if it is colliding with a sphere
			if (NewPosition.Distance(Ball_Position) <= BALL_RADIUS * 1.05) {
				double intersection_amount = BALL_RADIUS - NewPosition.Distance(Ball_Position);
				NewPosition.y += intersection_amount;
				Velocity.y = 0.0;
			}

			Position = NewPosition;
			OldPosition = temp;
		}
	}

	void Draw() {
		// Draw Particle
		glPushMatrix();
		glTranslatef(Position.x, Position.y, Position.z);
		glColor3f(PARTICLE_R, PARTICLE_G, PARTICLE_B);
		glutSolidSphere(PARTICLE_RADIUS, 25, 25); // draw the ball, but with a slightly lower radius, otherwise we could get ugly visual artifacts of rope penetrating the ball slightly
		glPopMatrix();
	}
};

  /////////////////////////////
 //		Spring Class		//
/////////////////////////////
//	Connects two masses with a rest length
//	Apply forces to masses when compressed or streched

class Spring {
public:
	Particle *P0; // left particle
	Particle *P1; // right particle

	double K;
	double D;

	double RestLength;

	Spring() {
		P0 = NULL;
		P1 = NULL;
		K = SPRING_K;
		D = SPRING_D;
	}

	Spring(Particle *Left, Particle* Right, double length, double k, double d) {
		P0 = Left;
		P1 = Right;
		RestLength = length;
		K = k;
		D = d;
	}

	void Solve() {
		// Math Theory from Physics for Game Developers pp 261-262
		Vec3 delta_velocity = P1->Velocity - P0->Velocity; // get relative velocity between the particles
		Vec3 spring_vector = P1->Position - P0->Position; // get the direction of spring force

		double X = spring_vector.Magnitude() - RestLength; // X in the equation F = -K * X

		// the magnitude of the spring force is K * X
		// get the unit vector of p1 to p0 to get direction
		// scale by the force magnitude
		Vec3 force_spring = spring_vector.Normalized() * K * X;

		// apply forces to each particle
		P0->Forces += force_spring * 0.5;
		P1->Forces -= force_spring * 0.5;
	}
};

// Rope Simulator Class
class RopeSimulator {
public:
	std::vector<Particle> Particles;
	std::vector<Spring> Springs;

	RopeSimulator() {
		ResetRope();
	}

	void ResetRope() {
		// SUGGESTION: Make worldspace length larger before increasing particle count! (avoids roundoff errors)
		double rope_increment = (double)ROPE_WORLDSPACE_LENGTH / ((double)NUM_PARTICLES - 1.0);
		double rope_half = (double)ROPE_WORLDSPACE_LENGTH / 2.0;
		
		double spring_length = rope_increment;

		Particles.clear();
		Springs.clear();
		// initialize particles
		for (size_t i = 0; i < NUM_PARTICLES; i++) {
			// create a particle from left to right of the rope
			Vec3 pos = { (i * rope_increment) - rope_half, ROPE_HEIGHT, 0.0 };
			Particle new_particle{ pos, PARTICLE_MASS, false };
			Particles.push_back(new_particle);
		}

		Particles[0].Anchored = true;
		Particles[NUM_PARTICLES - 1].Anchored = true;

		// initialize springs
		for (size_t i = 0; i < NUM_PARTICLES - 1; i++) {
			Spring new_spring{ &Particles[i], &Particles[i + 1], spring_length, SPRING_K, SPRING_D };
			Springs.push_back(new_spring);
		}
	}

	// reset the forces on the particles (dont let them accumulate)
	void ResetForces() {
		for (size_t i = 0; i < Particles.size(); i++) {
			Particles[i].Forces = Vec3(0.0, 0.0, 0.0);
		}
	}

	void Solve() {
		for (size_t i = 0; i < Springs.size(); i++) {
			Springs[i].Solve();
		}

		for (size_t i = 0; i < Particles.size(); i++) {
			Particles[i].Solve();
		}
	}

	void Integrate() {
		ResetForces();
		Solve();
	}

	void Draw() {
		glLineWidth(ROPE_WIDTH);
		glColor3f(PARTICLE_R, PARTICLE_B, PARTICLE_G);
		glBegin(GL_LINE_STRIP);
		Vec3 pos;
		for (size_t i = 0; i < Particles.size(); i++) {
			pos = Particles[i].Position;
			glVertex3d(pos.x, pos.y, pos.z);
		}
		glEnd();

		for (size_t i = 0; i < Particles.size(); i++)
			Particles[i].Draw();
	}
};

RopeSimulator ropeSimulation;

  /////////////////////////////////////////////////
 //					OpenGL Code					//
/////////////////////////////////////////////////

// Init the simulation
void init(void) {
	glShadeModel(GL_SMOOTH);
	glClearColor(0.2f, 0.2f, 0.4f, 0.5f);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_COLOR_MATERIAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	GLfloat lightPos[4] = { -1.0, 1.0, 0.5, 0.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, (GLfloat *)&lightPos);
	glEnable(GL_LIGHT1);
	GLfloat lightAmbient1[4] = { 0.0, 0.0,  0.0, 0.0 };
	GLfloat lightPos1[4] = { 1.0, 0.0, -0.2, 0.0 };
	GLfloat lightDiffuse1[4] = { 0.5, 0.5,  0.3, 0.0 };
	glLightfv(GL_LIGHT1, GL_POSITION, (GLfloat *)&lightPos1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, (GLfloat *)&lightAmbient1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, (GLfloat *)&lightDiffuse1);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
}


// Helper drawing functions
void DrawSky() {
	// Draw a gradient background
	glBegin(GL_POLYGON);

	glColor3f(0.8f, 0.8f, 1.0f);
	glVertex3f(-200.0f, -100.0f, -100.0f);
	glVertex3f(200.0f, -100.0f, -100.0f);

	glColor3f(0.4f, 0.4f, 0.8f);
	glVertex3f(200.0f, 100.0f, -100.0f);
	glVertex3f(-200.0f, 100.0f, -100.0f);

	glEnd();
	glEnable(GL_LIGHTING);
}

void DrawSphere() {
	// Draw collision ball
	glPushMatrix();
	glTranslatef(BALL_X, BALL_Y, BALL_Z);
	glColor3f(1.0f, 1.0f, 0.0f);
	glutSolidSphere(BALL_RADIUS - 0.1, 50, 50);
	glPopMatrix();
}

// Update the simulation and render it
void display(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glDisable(GL_LIGHTING);

	DrawSky();

	glTranslatef(0.0, 0.0, -ROPE_WORLDSPACE_LENGTH * 0.5); // keep camera focused on rope

	if (!PAUSED) ropeSimulation.Integrate();
	DrawSphere();
	ropeSimulation.Draw();

	glutSwapBuffers();
	glutPostRedisplay();
}



// "Boiler Plate"
void reshape(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (h == 0) {
		gluPerspective(80, (float)w, 1.0, 5000.0);
	}
	else {
		gluPerspective(80, (float)w / (float)h, 1.0, 5000.0);
	}
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 27: // escape
		exit(0);
		break;
	case 32: // space bar
		PAUSED = !PAUSED;
		break;
	case 'r': // r (reset)
		ropeSimulation.ResetRope();
		PAUSED = true;
	default:
		break;
	}
}

void arrow_keys(int a_keys, int x, int y) {
	switch (a_keys) {
	case GLUT_KEY_UP:
		glutFullScreen();
		break;
	case GLUT_KEY_DOWN:
		glutReshapeWindow(1280, 720);
		break;
	default:
		break;
	}
}

// Create the main loop
int main(int argc, char *argv[]) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(1280, 720);
	glutCreateWindow("R O P E   S I M U L A T I O N -- Chaz acheronti");
	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(arrow_keys);
	glutMainLoop();
}