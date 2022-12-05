#pragma once
#include "util/vectorbase.h"
#include "util/matrixbase.h"
#include "util/quaternion.h"
namespace GamePhysics {
	const Vec3 GRAVITY(0.f, -9.81f, 0.f);

	struct Rigidbody {
		Vec3 size;
		Vec3 position;
		Quat rotation;

		double mass;

		matrix4x4<double> inertia;
		matrix4x4<double> inverseInertia;

		Vec3 velocity;
		
		Vec3 torque;
		Vec3 angularMomentum;
		Vec3 angularVelocity;

		Vec3 force;

		void PrecomputeInertia() {
			constexpr double div = 1 / 12.0;
			
			double xi = div * mass * (size.y * size.y + size.z * size.z);
			double yi = div * mass * (size.x * size.x + size.z * size.z);
			double zi = div * mass * (size.y * size.y + size.x * size.x);
			
			inertia.initScaling(xi, yi, zi);
			inverseInertia.initScaling(1. / xi, 1. / yi, 1. / zi);
		}

		matrix4x4<double> GetCurrentInverseInertiaTensor() {
			auto rot = rotation.getRotMat();
			auto rotInv = rot;
			rotInv.transpose();

			return rotInv * inverseInertia * rot;
		}
	};

	struct Collision {
		Vec3 position;
		Vec3 normal;
		float depth;
		int rb1;
		int rb2;
	};

	static Vec3 ViewportToWorldpoint(XMMATRIX worldMat, XMMATRIX viewMat, Point2D mouse)
	{
		Mat4 worldViewInv = Mat4(worldMat * viewMat);
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3(mouse.x, mouse.y, 0);
		return worldViewInv.transformVectorNormal(inputView);
	}

	static Vec3 ToEulerAngles(Quat q) {
		Vec3 angles;

		// roll (x-axis rotation)
		double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
		double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
		angles.x = std::atan2(sinr_cosp, cosr_cosp);

		// pitch (y-axis rotation)
		double sinp = 2 * (q.w * q.y - q.z * q.x);
		if (std::abs(sinp) >= 1)
			angles.z = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		else
			angles.z = std::asin(sinp);

		// yaw (z-axis rotation)
		double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
		double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
		angles.y = std::atan2(siny_cosp, cosy_cosp);

		return angles;
	}
}
