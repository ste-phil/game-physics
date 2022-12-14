#include "MassSpringSystemSimulator.h"
#include "Integrators.h"

using MSSS = MassSpringSystemSimulator;

#pragma region UI Function
const char* MSSS::getTestCasesStr() { return "Demo2,Demo3,Demo4"; }

void MSSS::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Damping factor", TW_TYPE_FLOAT, &m_fDamping, "min=0.000 step=0.001");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity strength", TW_TYPE_FLOAT, &m_fGravityStrength, "min=0.0 step=0.1");

	TwAddVarRW(DUC->g_pTweakBar, "Ground Collider", TW_TYPE_BOOLCPP, &m_useGroundCollider, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &m_useGravity, "");

	TwAddVarRW(DUC->g_pTweakBar, "Ignore Mouse Input", TW_TYPE_BOOLCPP, &m_ignoreMouseInput, "");

	TwType TW_TYPE_INTEGRATORS = TwDefineEnumFromString("Integrators", "Euler,Leapfrog,Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATORS, &m_iIntegrator, "");
}

void MSSS::notifyCaseChanged(int testCase) {
	reset();

	switch (testCase) {
	case 0:
		m_iIntegrator = IntegrationMethod::Euler;
	case 1:
		m_iIntegrator = IntegrationMethod::Midpoint;

		setMass(10);
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

		setStiffness(40);
		addSpring(0, 1, 1);

		break;
	case 2:
		m_iIntegrator = IntegrationMethod::Leapfrog;


		setMass(10);
		setStiffness(80);
		const float defaultJointDistance = .4f;
		const float defaultLength = .29f;

		#pragma region CLOTH
		int massPointIdx = 0;
		for (int i = 0; i < 5; i++)
		{
			//Create Cloth
			for (int j = 0; j < 5; j++)
			{
				addMassPoint(Vec3(i * defaultJointDistance, 0, j * defaultJointDistance), Vec3(0, 0, 0), false);

				if (i - 1 >= 0)
					addSpring(massPointIdx, massPointIdx - 5, defaultLength);
				if (j - 1 >= 0)
					addSpring(massPointIdx, massPointIdx - 1, defaultLength);

				massPointIdx++;
			}
		}
		#pragma endregion

		#pragma region CUBE
		//Create Cube
		int p0 = addMassPoint(Vec3(0,0,0), Vec3(0,0,0), false);
		int p1 = addMassPoint(Vec3(0.25, 0, 0), Vec3(0, 0, 0), false);
		int p2 = addMassPoint(Vec3(-0.25, 0, 0), Vec3(0, 0, 0), false);
		int p3 = addMassPoint(Vec3(0, 0, 0.25), Vec3(0, 0, 0), false);
		int p4 = addMassPoint(Vec3(0, 0, -0.25), Vec3(0, 0, 0), false);
		int p5 = addMassPoint(Vec3(0, 0.25, 0), Vec3(0, 0, 0), true);
		int p6 = addMassPoint(Vec3(0, -0.25, 0), Vec3(0, 0, 0), false);

		addSpring(p0, p1, 0.25);
		addSpring(p0, p2, 0.25);
		addSpring(p0, p3, 0.25);
		addSpring(p0, p4, 0.25);
		addSpring(p0, p5, 0.25);
		addSpring(p0, p6, 0.25);

		addSpring(p1, p3, 0.5);
		addSpring(p1, p4, 0.5);
		addSpring(p1, p5, 0.5);
		addSpring(p1, p6, 0.5);

		addSpring(p2, p3, 0.5);
		addSpring(p2, p4, 0.5);
		addSpring(p2, p5, 0.5);
		addSpring(p2, p6, 0.5);

		addSpring(p3, p5, 0.5);
		addSpring(p3, p6, 0.5);
		addSpring(p4, p5, 0.5);
		addSpring(p4, p6, 0.5);
#pragma endregion

		break;
	}
}

void MSSS::onClick(int x, int y) {
	if (m_ignoreMouseInput) return;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
	m_mousePressed = true;
}

void MSSS::onMouse(int x, int y) {
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
	m_mousePressed = false;
}
#pragma endregion

#pragma region Simulation Functions

MSSS::MassSpringSystemSimulator() {
	setMass(1.f);
	setStiffness(.1f);
	setDampingFactor(.001f);
	m_fGravityStrength = 1.0f;

	m_iIntegrator = IntegrationMethod::Euler;
}

void MSSS::reset() {
	m_massPoints.clear();
	m_springs.clear();
}

void MSSS::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
	
	const float sphereSize = .01f;
	for (size_t i = 0; i < m_massPoints.size(); i++)
	{
		DUC->drawSphere(m_massPoints[i].position, Vec3(sphereSize));
	}

	for (size_t i = 0; i < m_springs.size(); i++)
	{
		auto mp1 = m_massPoints[m_springs[i].massPointIndex1];
		auto mp2 = m_massPoints[m_springs[i].massPointIndex2];

		DUC->beginLine();
		DUC->drawLine(mp1.position, Vec3(1), mp2.position, Vec3(1));
		DUC->endLine();
	}
	if (m_mousePressed && !m_ignoreMouseInput)
	{
		//Get Difference in Mouse Movement
		Point2D mouseDiff;
		mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
		mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
		//Get Position of Mouse Cursor in 3D space, scaled down
		Vec3 mouseVector = ViewportToWorldpoint(mouseDiff) * 0.001f;
		mouseVector.y *= -1;
		//Draw Mouse Vector as Visual Guide
		DUC->drawSphere(mouseVector, Vec3(sphereSize));
		DUC->beginLine();
		DUC->drawLine(Vec3(0,0,0), Vec3(1,1,0), mouseVector, Vec3(1,1,0));
		DUC->endLine();
	}
	
	
}


void MSSS::externalForcesCalculations(float timeElapsed) {
	//cout << "Simulation external " << timeElapsed << endl;

	m_externalForce = m_useGravity ? (GRAVITY * m_fGravityStrength) : Vec3(0, 0, 0);
}

void MSSS::simulateTimestep(float timeStep) {
	//cout << "Simulation internal " << timeStep << endl;

	for (size_t i = 0; i < m_springs.size(); i++)
	{
		auto spring = m_springs[i];

		//calculate spring force
		auto& mp1 = m_massPoints[spring.massPointIndex1];
		auto& mp2 = m_massPoints[spring.massPointIndex2];
		
		auto force = calculateElasticForce(mp1, mp2, spring);
		auto externalForce = m_externalForce;

		auto acc1 = (-force + externalForce) / mp1.mass;
		auto acc2 = (force + externalForce) / mp1.mass;
		
		auto worldInput = calculateWorldInput();
		/*cout << "Length: " << length << endl;
		cout << "Force: " << force << endl;
		cout << "Pos1: " << mp1.position << ", Acc1: " << acc1 << endl;
		cout << "Pos2: " << mp2.position << ", Acc2: " << acc2 << endl;*/
		if(!mp1.isFixed()) integratePosition(acc1, mp1.velocity, mp1.position, mp2.position, spring, mp1.mass, timeStep);			
		if(!mp2.isFixed()) integratePosition(acc2, mp2.velocity, mp2.position, mp1.position, spring, mp2.mass, timeStep);
		mp1.position += worldInput;
		mp1.position += worldInput;
	}

	for (size_t i = 0; i < m_massPoints.size(); i++)
	{
		auto& mp = m_massPoints[i];

		mp.velocity *= (1 - m_fDamping);

		const float groundPlaneY = -1.0f;
		const float bounceDamping = .5f;
		if (m_useGroundCollider && mp.position.y < groundPlaneY) {
			mp.velocity.y = -mp.velocity.y * bounceDamping;
			mp.position.y = groundPlaneY;
		}

	}
}


void MSSS::integratePosition(Vec3 acceleration, Vec3& velocity, Vec3& position, Vec3 otherPos, 
		const Spring& spring, const float mass, float dt) {
	switch (m_iIntegrator)
	{
	case EULER:
		Integrators::ExplicitEuler::integratePositionVelocity(acceleration, velocity, position, dt);
		break;
	case MIDPOINT:
		Integrators::Midpoint::integratePositionVelocity(acceleration, velocity, position, otherPos, spring, mass, dt, m_externalForce);
		break;
	case LEAPFROG:
		Integrators::LeapFrog::integratePositionVelocity(acceleration, velocity, position, dt);
		break;
	}
}

Vec3 MSSS::calculateElasticForce(const MassPoint& mp1, const MassPoint& mp2, const Spring& spring) {
	auto dir = mp2.position - mp1.position;
	auto length = norm(dir);
	auto normDir = dir / length;

	auto force = -spring.stiffness * (length - spring.restLength) * normDir;

	return force;
}

Vec3 MSSS::calculateWorldInput() 
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Vec3 inputWorld = ViewportToWorldpoint(mouseDiff);
		// find a proper scale!
		float inputScale = 0.000001f;
		inputWorld = inputWorld * inputScale;
		return inputWorld;
	}
	return Vec3(0, 0, 0);
}

Vec3 MSSS::ViewportToWorldpoint(Point2D mouse)
{
	Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
	worldViewInv = worldViewInv.inverse();
	Vec3 inputView = Vec3(mouse.x, mouse.y, 0);
	return worldViewInv.transformVectorNormal(inputView);
}
#pragma endregion

// Specific Functions
void MSSS::setMass(float mass) { this->m_fMass = mass; }
void MSSS::setStiffness(float stiffness) { this->m_fStiffness = stiffness; }
void MSSS::setDampingFactor(float damping) { this->m_fDamping = damping; }



int MSSS::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	MassPoint p = isFixed ? MassPoint(position) : MassPoint(m_fMass, Velocity, position);
	m_massPoints.push_back(p);

	return (int) (m_massPoints.size() - 1);
}

void MSSS::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring s(m_fStiffness, initialLength, masspoint1, masspoint2);
	m_springs.push_back(s);
}

int MSSS::getNumberOfMassPoints() {
	return (int) m_massPoints.size();
}

int MSSS::getNumberOfSprings() {
	return (int) m_springs.size();
}

Vec3 MSSS::getPositionOfMassPoint(int index) {
	return m_massPoints[index].position;
}

Vec3 MSSS::getVelocityOfMassPoint(int index) {
	return m_massPoints[index].velocity;
}

void MSSS::applyExternalForce(Vec3 force) {

}
