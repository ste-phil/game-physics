#ifndef GAMEPHYSICS_INTEGRATORS_H
#define GAMEPHYSICS_INTEGRATORS_H
#include "util/vectorbase.h"

namespace GamePhysics {
	namespace Integrators {
		namespace ExplicitEuler {
			void integratePositionVelocity(Vec3 acc, Vec3& vel, Vec3& pos, float dt) {
				vel = vel + acc * dt;
				pos = pos + vel * dt;
			}
		}

		namespace LeapFrog {
			void integratePositionVelocity(Vec3 acc, Vec3& vel, Vec3& pos, float dt) {
				pos = pos + vel * dt + .5f * acc * dt * dt;
				vel = vel + .5f * (acc + acc * dt) * dt;
			}
		}

		namespace Midpoint {
			void integratePositionVelocity(Vec3 acc, Vec3& vel, Vec3& pos, Vec3 otherPos, 
					const Spring& spring, const float mass, float dt) {

				auto xTmp = pos + vel * dt * .5f;

				auto vTmp = vel + acc * .5f * dt;
				pos = pos + vTmp * dt;

				//calculate new elastic forces at xTmp and vTmp
				auto dir = otherPos - pos;
				auto length = norm(dir);
				auto normDir = dir / length;

				auto force = -spring.stiffness * (length - spring.restLength) * normDir;
				auto aTmp = force / mass;

				vel = vel + (acc + aTmp) * dt;
			}
		}
	}
}

#endif