#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "Entities.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

using namespace GamePhysics::Entities;

class MassSpringSystemSimulator: public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void notifyCaseChanged(int testCase);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Simulation Funcion
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);


	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = static_cast<IntegrationMethod>(integrator);
	}

private:
	//Integrate position according to chosen integration method
	void integratePosition(Vec3 acceleration, Vec3& velocity, Vec3& position, Vec3 otherPos,
		const Spring& spring, const float mass, float dt);
	Vec3 calculateElasticForce(const MassPoint& mp1, const MassPoint& mp2, const Spring& spring);
	
	Vec3 calculateWorldInput();
	Vec3 ViewportToWorldpoint(Point2D mouse);

	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	float m_fGravityStrength;
	IntegrationMethod m_iIntegrator;

	bool m_useGravity;
	bool m_useGroundCollider;

	bool m_ignoreMouseInput;

	std::vector<MassPoint> m_massPoints;
	std::vector<Spring> m_springs;

	bool m_mousePressed;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif