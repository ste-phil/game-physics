#include "RigidBodySystemSimulator.h"

using RBSS = RigidBodySystemSimulator;

RBSS::RigidBodySystemSimulator(){

}


// Functions
const char* RBSS::getTestCasesStr(){}
void RBSS::initUI(DrawingUtilitiesClass* DUC){
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Damping factor", TW_TYPE_FLOAT, &m_fDamping, "min=0.000 step=0.001");

	TwAddVarRW(DUC->g_pTweakBar, "Ground Collider", TW_TYPE_BOOLCPP, &m_useGroundCollider, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &m_useGravity, "");

	TwType TW_TYPE_INTEGRATORS = TwDefineEnumFromString("Integrators", "Euler,Leapfrog,Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATORS, &m_iIntegrator, "");
}
void RBSS::reset(){
	rigidbodies.clear();
}
void RBSS::drawFrame(ID3D11DeviceContext* pd3dImmediateContext){
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));

	const float sphereSize = .01f;

	for (size_t i = 0; i < rigidbodies.size(); i++)
	{
		auto h = rigidbodies[i].size / 2;

		auto bottomFrontLeft = rigidbodies[i].position + Vec3(-h.x, -h.y, -h.z);
		auto bottomBackLeft = rigidbodies[i].position + Vec3(-h.x, -h.y, h.z);
		auto bottomFrontRight = rigidbodies[i].position + Vec3(h.x, -h.y, h.z);
		auto bottomBackRight = rigidbodies[i].position + Vec3(h.x, -h.y, -h.z);
		auto topBackLeft = rigidbodies[i].position + Vec3(-h.x, h.y, -h.z);
		auto topBackRight = rigidbodies[i].position + Vec3(h.x, h.y, -h.z);
		auto topFrontRight = rigidbodies[i].position + Vec3(h.x, h.y, h.z);
		auto topFrontLeft = rigidbodies[i].position + Vec3(-h.x, h.y, h.z);

		DUC->drawSphere(bottomFrontLeft, Vec3(sphereSize));
		DUC->drawSphere(bottomBackLeft, Vec3(sphereSize));
		DUC->drawSphere(bottomFrontRight, Vec3(sphereSize));
		DUC->drawSphere(bottomBackRight, Vec3(sphereSize));
		DUC->drawSphere(topBackLeft, Vec3(sphereSize));
		DUC->drawSphere(topBackRight, Vec3(sphereSize));
		DUC->drawSphere(topFrontRight, Vec3(sphereSize));
		DUC->drawSphere(topFrontLeft, Vec3(sphereSize));

		DUC->beginLine();
		DUC->drawLine(topBackLeft, Vec3(1), topBackRight, Vec3(1));
		DUC->endLine();
		DUC->beginLine();
		DUC->drawLine(topBackRight, Vec3(1), topFrontRight, Vec3(1));
		DUC->endLine();
		DUC->beginLine();
		DUC->drawLine(topFrontRight, Vec3(1), topFrontLeft, Vec3(1));
		DUC->endLine();
		DUC->beginLine();
		DUC->drawLine(topFrontLeft, Vec3(1), topBackLeft, Vec3(1));
		DUC->endLine();


		DUC->beginLine();
		DUC->drawLine(bottomBackLeft, Vec3(1), bottomBackRight, Vec3(1));
		DUC->endLine();
		DUC->beginLine();
		DUC->drawLine(bottomBackRight, Vec3(1), bottomFrontRight, Vec3(1));
		DUC->endLine();
		DUC->beginLine();
		DUC->drawLine(bottomFrontRight, Vec3(1), bottomFrontLeft, Vec3(1));
		DUC->endLine();
		DUC->beginLine();
		DUC->drawLine(bottomFrontLeft, Vec3(1), bottomBackLeft, Vec3(1));
		DUC->endLine();



		DUC->drawLine(bottomFrontLeft, Vec3(1), topFrontLeft, Vec3(1));
		DUC->drawLine(bottomBackLeft, Vec3(1), topBackLeft, Vec3(1));
		DUC->drawLine(bottomFrontRight, Vec3(1), topFrontRight, Vec3(1));
		DUC->drawLine(bottomBackRight, Vec3(1), topBackRight, Vec3(1));

	}
}
void RBSS::notifyCaseChanged(int testCase){
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
void RBSS::externalForcesCalculations(float timeElapsed){
	//cout << "Simulation external " << timeElapsed << endl;

	m_externalForce = m_useGravity ? GRAVITY : Vec3(0, 0, 0);
}
void RBSS::simulateTimestep(float timeStep){
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
void RBSS::onClick(int x, int y){}
void RBSS::onMouse(int x, int y){}

// ExtraFunctions
int RBSS::getNumberOfRigidBodies() {
	return rigidbodies.size();
}
Vec3 RBSS::getPositionOfRigidBody(int i) {
	return rigidbodies[i].position;
}
Vec3 RBSS::getLinearVelocityOfRigidBody(int i){
	return rigidbodies[i].velocity;
}
Vec3 RBSS::getAngularVelocityOfRigidBody(int i){
	return rigidbodies[i].angularVelocity;

}
void RBSS::applyForceOnBody(int i, Vec3 loc, Vec3 force){

}
void RBSS::addRigidBody(Vec3 position, Vec3 size, int mass){
	Rigidbody rb;
	rb.position = position;
	rb.size = size;
	rb.mass = mass;

	rigidbodies.push_back(rb);
}
void RBSS::setOrientationOf(int i, Quat orientation){
	rigidbodies[i].rotation = orientation;
}
void RBSS::setVelocityOf(int i, Vec3 velocity) {
	rigidbodies[i].velocity = velocity;
}