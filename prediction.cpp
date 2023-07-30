#include "prediction.h"
#include <iostream>
namespace easy_win
{
	namespace prediction
	{
		void __cdecl OnNewPath(api::object::GameObject* object, Vector3** path, int size, float* dash_speed, bool is_not_dash, int unk1, int unk2, int unk3, int unk4, int unk5, int unk6, int unk7)
		{
			if (size >= 2 && object->GetTeam() != (*api::object_manager::local_player)->GetTeam() && api::functions::IsHero(object) && object->IsAlive() && api::functions::IsTargetable(object))
			{
				if (is_not_dash && object->GetAIManager()->IsMoving() && !object->GetAIManager()->IsDashing())
				{
					Vector3 prev_direction = object->GetAIManager()->TargetPosition().ToXZ() - object->GetPosition().ToXZ();
					Vector3 new_direction = (*path)[size - 1].ToXZ() - object->GetPosition().ToXZ();

					if (math::AngleBetween(prev_direction, new_direction).value_or(0.0) >= 90.0f)
					{
						event_manager::Trigger(LeagueEvents::PredHighChance, object, *path, size, dash_speed, is_not_dash);
					}
				}
				else
				{
					event_manager::Trigger(LeagueEvents::PredHighChance, object, *path, size, dash_speed, is_not_dash);
				}
			}
		}

		std::optional<Vector3> Intercept(const Vector3& src, const Vector3& dst, const Vector3& dst_velocity, const float missile_speed, const float start_time)
		{
			//Given source position, target position, target velocity and missile speed
			//if missile can collide with target, returns collision position
			//otherwise returns std::nullopt
			//

			/*const double tx = static_cast<double>(dst.x) - static_cast<double>(src.x);
			const double ty = static_cast<double>(dst.z) - static_cast<double>(src.z);
			const double tvx = dst_velocity.x;
			const double tvy = dst_velocity.z;

			const double a = tvx * tvx + tvy * tvy - static_cast<double>(missile_speed) * static_cast<double>(missile_speed);
			const double b = 2 * (tvx * tx + tvy * ty);
			const double c = tx * tx + ty * ty;*/

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
					std::optional<Vector3> interception{ {0, 0, 0} };
					float x = dst.x + dst_velocity.x * t;
					float y = 0;
					float z = dst.z + dst_velocity.z * t;
					//api::functions::GetHeightAt(x, z, &y);
					interception.value().x = x;
					interception.value().y = y;
					interception.value().z = z;
					return interception;
				}
			}

			return std::nullopt;
		}

		std::optional<Vector3> SegmentPredict(const Vector3& src, const Vector3& dst_origin, const Vector3& dst_target, const float missile_speed, const float speed, const float cast_delay, const float hitbox, const float start_time)
		{
			Vector3 target_pos = dst_origin.Extend(dst_target, math::Distance(speed, cast_delay));

			if ((target_pos - dst_origin).Mod() <= (dst_origin - dst_target).Mod() && (target_pos - dst_target).Mod() <= (dst_origin - dst_target).Mod())
			{
				target_pos = target_pos.Extend(dst_target, -1 * hitbox);

				std::optional<Vector3> interception = Intercept(src, target_pos, target_pos.Velocity(dst_target, speed), missile_speed, start_time);

				if (interception.has_value())
				{
					if ((dst_target - target_pos).Mod() /* + hitbox???*/ >= (interception.value() - target_pos).Mod())
					{
						return interception;
					}
				}
			}

			return std::nullopt;
		}

		std::optional<Vector3> SmartPredict(const Vector3& origin, const api::object::GameObject* target, const float hitbox, const float missile_speed, float cast_delay, const float range, const bool edge_collition, const bool extend)
		{
			AIManager* target_ai_manager = target->GetAIManager();
			float target_hitbox = target->GetBoundingRadius() * edge_collition;
			Vector3 src = origin.ToXZ();
			cast_delay = cast_delay + math::MillisecondToSecond(api::functions::GetPing());

			if (target_ai_manager->IsMoving())
			{
				if (!target_ai_manager->IsDashing())
				{
					//Target is walking
					//
					Vector3* waypoints = target_ai_manager->GetWaypoints();
					float time = 0;
					Vector3 dest_origin = target_ai_manager->ServerPosition().ToXZ();
					float target_speed = target->GetSpeed();
					float segment_time = 0;
					Vector3 dest_target = Vector3(0, 0, 0);
					Vector3 dest_vec = Vector3(0, 0, 0);
					bool init = false;
					int passed_waypoints = target_ai_manager->PassedWaypoints();
					int waypoints_size = target_ai_manager->Size();
					std::optional<int> target_path;

					for (int i = passed_waypoints; i < waypoints_size; i++)
					{
						dest_target = (waypoints)[i].ToXZ();
						dest_vec = (dest_target - dest_origin);

						std::optional<float> t = math::Time(dest_vec.Mod(), target_speed);

						if (t.has_value())
						{
							segment_time += t.value();

							if (segment_time >= cast_delay)
							{
								dest_origin = dest_origin.Extend(dest_target, math::Distance(target_speed, cast_delay - time));
								target_path = i;
								break;
							}

							time += t.value();
						}

						dest_origin = dest_target;
					}

					if (target_path.has_value())
					{
						time = 0.0f;

						for (int i = target_path.value(); i < waypoints_size; i++)
						{
							dest_target = (waypoints)[i].ToXZ();
							dest_vec = (dest_target - dest_origin);

							std::optional<float> t = math::Time(dest_vec.Mod(), target_speed);

							if (t.has_value())
							{
								std::optional<Vector3> interception = SegmentPredict(src, dest_origin, dest_target, missile_speed, target_speed, 0.0f, hitbox + target_hitbox, time);

								if (interception.has_value() && (interception.value().ToXZ() - src).Mod() <= range)
								{
									return interception;
								}

								time += t.value();
							}

							dest_origin = dest_target;
						}
					}
				}
				else
				{
					//Target is dashing
					//
					Vector3 src = (*api::object_manager::local_player)->GetAIManager()->ServerPosition().ToXZ();
					Vector3 target_origin = target_ai_manager->ServerPosition().ToXZ();
					Vector3 target_end = target_ai_manager->TargetPosition().ToXZ();

					std::optional interception = SegmentPredict(src, target_origin, target_end, missile_speed, target_ai_manager->DashSpeed(), cast_delay, 0.0f, 0.0f);

					if (interception.has_value() && (interception.value().ToXZ() - src).Mod() <= range)
					{
						return interception;
					}
				}
			}
			else
			{
				//Target is not moving
				//
				Vector3 target_pos = target_ai_manager->ServerPosition().ToXZ();

				float missing_distance = std::min(std::max(static_cast<float>((target_pos - src).Mod() - range), 0.0f), hitbox + target_hitbox);

				Vector3 interception = target_pos.Extend(src, missing_distance * extend);

				if ((interception.ToXZ() - src).Mod() <= range)
				{
					interception.y = target_ai_manager->ServerPosition().y;
					return interception;
				}
			}

			return std::nullopt;
		}

		std::optional<Vector3> NoMissilePredict(const Vector3& origin, const api::object::GameObject* target, const float hitbox, float cast_delay, const float range, const bool edge_collition, const bool extend)
		{
			AIManager* target_ai_manager = target->GetAIManager();
			float target_hitbox = target->GetBoundingRadius() * edge_collition;
			Vector3 src = origin.ToXZ();
			cast_delay = cast_delay + math::MillisecondToSecond(api::functions::GetPing());

			if (target_ai_manager->IsMoving())
			{
				if (!target_ai_manager->IsDashing())
				{
					Vector3* waypoints = target_ai_manager->GetWaypoints();
					Vector3 dest_target = Vector3(0, 0, 0);
					Vector3 dest_vec = Vector3(0, 0, 0);
					Vector3 dest_origin = target_ai_manager->ServerPosition().ToXZ();
					float time = 0;
					float segment_time = 0;
					int passed_waypoints = target_ai_manager->PassedWaypoints();
					int waypoints_size = target_ai_manager->Size();

					for (int i = passed_waypoints; i < waypoints_size; i++)
					{
						dest_target = (waypoints)[i].ToXZ();

						dest_vec = (dest_target - dest_origin);

						std::optional<float> t = math::Time(dest_vec.Mod(), target->GetSpeed());

						if (t.has_value())
						{
							segment_time += t.value();

							if (segment_time >= cast_delay)
							{
								Vector3 predicted_pos = dest_origin.Extend(dest_target, math::Distance(target->GetSpeed(), cast_delay - time) - target_hitbox - hitbox);

								if ((predicted_pos.ToXZ() - src).Mod() <= range)
								{
									return std::optional<Vector3> { predicted_pos };
								}
							}

							time += t.value();
						}

						dest_origin = dest_target;
					}
				}
				else
				{
					//Target is dashing
					//
					Vector3 target_origin = target_ai_manager->ServerPosition().ToXZ();
					Vector3 target_end = target_ai_manager->TargetPosition().ToXZ();

					if (math::Distance(target_ai_manager->DashSpeed(), cast_delay - math::Time(hitbox + target_hitbox, target->GetSpeed()).value_or(range)) <= (target_end - target_origin).Mod())
					{
						Vector3 predicted_pos = target_origin.Extend(target_end, std::min(math::Distance(target_ai_manager->DashSpeed(), cast_delay), static_cast<float>((target_end - target_origin).Mod())));

						if ((predicted_pos.ToXZ() - src).Mod() <= range)
						{
							return std::optional<Vector3> { predicted_pos };
						}
					}
				}
			}
			else
			{
				//Target is not moving
				//
				Vector3 target_pos = target_ai_manager->ServerPosition().ToXZ();

				float missing_distance = std::min(std::max(static_cast<float>((target_pos - src).Mod() - range), 0.0f), hitbox + target_hitbox);

				Vector3 interception = target_pos.Extend(src, missing_distance * extend);

				if ((interception - src).Mod() <= range)
				{
					interception.y = target_ai_manager->ServerPosition().y;
					return interception;
				}
			}

			return std::nullopt;
		}

		std::optional<Vector3> PredictMissileOnNewPath(const Vector3& origin, const api::object::GameObject* target, const float hitbox, const float missile_speed, float cast_delay, const float range, Vector3* waypoints, int size, float* dash_speed, bool is_not_dash, const bool edge_collition)
		{
			float target_hitbox = target->GetBoundingRadius() * edge_collition;
			Vector3 src = origin.ToXZ();
			cast_delay = cast_delay + math::MillisecondToSecond(api::functions::GetPing());

			if (is_not_dash)
			{
				//Target is walking
				//
				float time = 0;
				Vector3 dest_origin = (waypoints)[0].ToXZ();
				float target_speed = target->GetSpeed();
				float segment_time = 0;
				Vector3 dest_target = Vector3(0, 0, 0);
				Vector3 dest_vec = Vector3(0, 0, 0);

				std::optional<int> target_path;

				for (int i = 1; i < size; i++)
				{
					dest_target = (waypoints)[i].ToXZ();
					dest_vec = (dest_target - dest_origin);

					std::optional<float> t = math::Time(dest_vec.Mod(), target_speed);

					if (t.has_value())
					{
						segment_time += t.value();
						if (segment_time >= cast_delay)
						{
							dest_origin = dest_origin.Extend(dest_target, math::Distance(target_speed, cast_delay - time));
							target_path = i;
							break;
						}

						time += t.value();
					}

					dest_origin = dest_target;
				}

				if (target_path.has_value())
				{
					time = 0.0f;

					for (int i = target_path.value(); i < size; i++)
					{
						dest_target = (waypoints)[i].ToXZ();
						dest_vec = (dest_target - dest_origin);

						std::optional<float> t = math::Time(dest_vec.Mod(), target_speed);

						if (t.has_value())
						{
							std::optional<Vector3> interception = SegmentPredict(src, dest_origin, dest_target, missile_speed, target_speed, 0.0f, hitbox + target_hitbox, time);

							if (interception.has_value() && (interception.value().ToXZ() - src).Mod() <= range)
							{
								return interception;
							}

							time += t.value();
						}

						dest_origin = dest_target;
					}
				}
			}
			else
			{
				//Target is dashing
				//
				Vector3 src = (*api::object_manager::local_player)->GetAIManager()->ServerPosition().ToXZ();
				Vector3 target_origin = waypoints[0].ToXZ();
				Vector3 target_end = waypoints[size - 1].ToXZ();

				std::optional interception = SegmentPredict(src, target_origin, target_end, missile_speed, *dash_speed, cast_delay, 0.0f, 0.0f);

				if (interception.has_value() && (interception.value().ToXZ() - src).Mod() <= range)
				{
					return interception;
				}
			}

			return std::nullopt;
		}

		std::optional<Vector3> PredictOnNewPath(const Vector3& origin, const api::object::GameObject* target, const float hitbox, float cast_delay, const float range, Vector3* waypoints, int size, float* dash_speed, bool is_not_dash, const bool edge_collition)
		{
			float target_hitbox = target->GetBoundingRadius() * edge_collition;
			Vector3 src = origin.ToXZ();
			cast_delay = cast_delay + math::MillisecondToSecond(api::functions::GetPing());

			if (is_not_dash)
			{
				Vector3 dest_target = Vector3(0, 0, 0);
				Vector3 dest_vec = Vector3(0, 0, 0);
				Vector3 dest_origin = (waypoints)[0].ToXZ();
				float time = 0;
				float segment_time = 0;

				for (int i = 1; i < size; i++)
				{
					dest_target = (waypoints)[i].ToXZ();

					dest_vec = (dest_target - dest_origin);

					std::optional<float> t = math::Time(dest_vec.Mod(), target->GetSpeed());

					if (t.has_value())
					{
						segment_time += t.value();

						if (segment_time >= cast_delay)
						{
							Vector3 predicted_pos = dest_origin.Extend(dest_target, math::Distance(target->GetSpeed(), cast_delay - time) - target_hitbox - hitbox);

							if ((predicted_pos.ToXZ() - src).Mod() <= range)
							{
								return std::optional<Vector3> { predicted_pos };
							}
						}

						time += t.value();
					}

					dest_origin = dest_target;
				}
			}
			else
			{
				//Target is dashing
				//
				Vector3 target_origin = waypoints[0].ToXZ();
				Vector3 target_end = waypoints[size - 1].ToXZ();

				if (math::Distance(*dash_speed, cast_delay - math::Time(hitbox + target_hitbox, target->GetSpeed()).value_or(range)) <= (target_end - target_origin).Mod())
				{
					Vector3 predicted_pos = target_origin.Extend(target_end, std::min(math::Distance(*dash_speed, cast_delay), static_cast<float>((target_end - target_origin).Mod())));

					if ((predicted_pos.ToXZ() - src).Mod() <= range)
					{
						return std::optional<Vector3> { predicted_pos };
					}
				}
			}

			return std::nullopt;
		}

		bool CheckLineMinionCollision(const Vector3& start_pos, const Vector3& end_pos, const float width, const float extra_move_hitbox)
		{
			Vector3 start = start_pos.ToXZ();
			Vector3 end = end_pos.ToXZ();

			Vector3 target_pos = Vector3(0, 0, 0);
			Vector3 origin_pos = Vector3(0, 0, 0);
			Vector3 predicted_pos = Vector3(0, 0, 0);

			for (size_t i = 0; i < (*api::object_manager::minions)->length; i++)
			{
				api::object::MinionObject* minion = (*api::object_manager::minions)->list[i];

				if (minion->IsValid() && minion->GetTeam() != (*api::object_manager::local_player)->GetTeam() && minion->GetHealth() > 0 && minion->IsVisible())
				{
					AIManager* ai_manager = minion->GetAIManager();
					float point_radius = minion->GetBoundingRadius();

					if (ai_manager->IsMoving() && false)
					{
						//Advanced predict collition con missilespeed y toda la cosa... usar la formula que iba a usar en 
						// el predicion para ver si iban a colisionar
						//predicted_pos = origin_pos.Extend(target_pos);

						//math::CheckLineCollision(ai_manager->ServerPosition()->ToXZ(), start_pos, end_pos, width, point_radius + extra_move_hitbox);
					}
					else
					{
						if (math::CheckLineCollision(ai_manager->ServerPosition().ToXZ(), start, end, width, point_radius))
						{
							return true;
						}
					}
				}
			}

			return false;
		}
	}
}