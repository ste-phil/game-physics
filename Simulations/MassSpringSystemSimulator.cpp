#include "MassSpringSystemSimulator.h"
#include "Integrators.h"

using MSSS = MassSpringSystemSimulator;

#pragma region UI Function
const char* MSSS::getTestCasesStr() { return "Demo1,Demo2,Demo3,Demo4"; }

void MSSS::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Damping factor", TW_TYPE_FLOAT, &m_fDamping, "min=0.000 step=0.001");

	TwAddVarRW(DUC->g_pTweakBar, "Ground Collider", TW_TYPE_BOOLCPP, &m_useGroundCollider, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &m_useGravity, "");

	TwType TW_TYPE_INTEGRATORS = TwDefineEnumFromString("Integrators", "Euler,Leapfrog,Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATORS, &m_iIntegrator, "");
}

void MSSS::notifyCaseChanged(int testCase) {
	reset();

	switch (testCase) {
	case 0:

		break;
	case 1:
		m_iIntegrator = IntegrationMethod::Euler;
	case 2:
		m_iIntegrator = IntegrationMethod::Midpoint;

		setMass(10);
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

		setStiffness(40);
		addSpring(0, 1, 1);

		break;
	case 3:
		m_iIntegrator = IntegrationMethod::Leapfrog;


		setMass(10);
		setStiffness(40);
		const float defaultJointDistance = .4f;
		const float defaultLength = .29f;

		int massPointIdx = 0;
		for (int i = 0; i < 5; i++)
		{
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


		break;
	}
}

void MSSS::onClick(int x, int y) {

}

void MSSS::onMouse(int x, int y) {

}
#pragma endregion

#pragma region Simulation Functions

MSSS::MassSpringSystemSimulator() {
	setMass(1.f);
	setStiffness(.1f);
	setDampingFactor(.1f);
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
}


void MSSS::externalForcesCalculations(float timeElapsed) {
	//cout << "Simulation external " << timeElapsed << endl;

	m_externalForce = m_useGravity ? GRAVITY : Vec3(0, 0, 0);
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

		/*cout << "Length: " << length << endl;
		cout << "Force: " << force << endl;
		cout << "Pos1: " << mp1.position << ", Acc1: " << acc1 << endl;
		cout << "Pos2: " << mp2.position << ", Acc2: " << acc2 << endl;*/

		integratePosition(acc1, mp1.velocity, mp1.position, mp2.position, spring, mp1.mass, timeStep);
		integratePosition(acc2, mp2.velocity, mp2.position, mp1.position, spring, mp2.mass, timeStep);
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
		Integrators::Midpoint::integratePositionVelocity(acceleration, velocity, position, otherPos, spring, mass, dt);
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
