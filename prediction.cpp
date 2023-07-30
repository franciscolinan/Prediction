#include "prediction.h"

namespace prediction
{
	auto Intercept(const Vector3& src, const Vector3& dst, const Vector3& dst_velocity, const float missile_speed, const float start_time) -> std::optional<Vector3>
	{
		//Given source position, target position, target velocity and missile speed
		//if missile can collide with target, returns collision position
		//otherwise returns std::nullopt
		//
		
		const long double a = dst_velocity.x * dst_velocity.x + dst_velocity.z * dst_velocity.z - missile_speed * missile_speed;
		const long double b = 2 * (dst.x * dst_velocity.x - src.x * dst_velocity.x + dst.z * dst_velocity.z - src.z * dst_velocity.z - (missile_speed * missile_speed * start_time));
		const long double c = dst.x * dst.x + dst.z * dst.z + src.x * src.x + src.z * src.z - 2 * (dst.x * src.x + dst.z * src.z) - (missile_speed * missile_speed * start_time * start_time);

		std::optional<long double> x1, x2;

		//If has solution
		//
		if (math::SolveQuadratic(a, b, c, x1, x2))
		{
			double t = 0.0;

			//If only one solution
			//
			if (x1.has_value() xor x2.has_value())
			{
				//t = solution
				//
				t = x1.has_value() ? x1.value() : x2.value();
			}
			//If two solutions or infinite solutions
			//
			else
			{
				//t = 0.0 when infinite solutions
				//t = min positive solution when two solutions
				//
				t = std::min(x1.value_or(0.0), x2.value_or(0.0));

				if (t < 0)
				{
					t = std::max(x1.value_or(0.0), x2.value_or(0.0));
				}
			}

			if (t >= 0)
			{
				std::optional<Vector3> interception{ { dst.x + dst_velocity.x * t, 0, dst.z + dst_velocity.z * t} };
				return interception;
			}
		}

		return std::nullopt;
	}
}
