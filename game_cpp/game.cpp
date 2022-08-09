
#define _USE_MATH_DEFINES
#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

#include "../framework/scene.hpp"
#include "../framework/game.hpp"


//-------------------------------------------------------
//	game parameters
//-------------------------------------------------------

namespace params
{
	namespace ship
	{
		constexpr float LINEAR_SPEED = 0.5f;
		constexpr float ANGULAR_SPEED = 0.5f;

		// Actually, this value should be calculated via "collider" size or mesh size, but framework doesn't provide this info
		constexpr float SIZE = 0.2f;

		constexpr float REFILL_TIME = 10.f;
	}

	namespace aircraft
	{
		constexpr float TARGET_RADIUS = 1.5f;

		constexpr float LINEAR_ACCELERATION = 0.3f;
		constexpr float LINEAR_SPEED = 2.5f;

		constexpr float ANGULAR_SPEED = 2.5f;

		constexpr float TAKEOFF_TIME = 3.f;
		constexpr float LIVE_TIME = 50.f;

		constexpr float LANDING_SPEED = LINEAR_SPEED / 1.5f;
	}
}

//-------------------------------------------------------
//	Basic Vector2 class
//-------------------------------------------------------

class Vector2
{
public:
	float x;
	float y;

	Vector2();
	Vector2( float vx, float vy );
	Vector2( Vector2 const &other );

	float get_length() const;
	Vector2 get_normalized() const;
	Vector2 get_rotated(float angle_rad) const;

	static float dot(const Vector2 & lhv, const Vector2 & rhv);
	static float angle_rad(const Vector2& lhv, const Vector2& rhv);
};

Vector2::Vector2() :
	x( 0.f ),
	y( 0.f )
{
}

Vector2::Vector2( float vx, float vy ) :
	x( vx ),
	y( vy )
{
}

Vector2::Vector2( Vector2 const &other ) :
	x( other.x ),
	y( other.y )
{
}

float Vector2::get_length() const
{
	return std::sqrt(x * x + y * y);
}

Vector2 Vector2::get_normalized() const
{
	float length = get_length();
	return Vector2(x / length, y / length);
}

float Vector2::dot(const Vector2& lhv, const Vector2& rhv)
{
	Vector2 normalized_lhv = lhv.get_normalized();
	Vector2 normalized_rhv = rhv.get_normalized();
	return normalized_lhv.x * normalized_rhv.x + normalized_lhv.y * normalized_rhv.y;
}

float Vector2::angle_rad(const Vector2& lhv, const Vector2& rhv)
{
	Vector2 normalized_lhv = lhv.get_normalized();
	Vector2 normalized_rhv = rhv.get_normalized();
	return -std::atan2(normalized_lhv.x * normalized_rhv.y - normalized_lhv.y * normalized_rhv.x, normalized_lhv.x * normalized_rhv.x + normalized_lhv.y * normalized_rhv.y);
}

Vector2 Vector2::get_rotated(float angle_rad) const
{
	return Vector2(std::cos(angle_rad) * x - std::sin(angle_rad) * y, std::sin(angle_rad) * x + std::cos(angle_rad) * y);
}

Vector2 operator + ( Vector2 const &left, Vector2 const &right )
{
	return Vector2( left.x + right.x, left.y + right.y );
}

Vector2 operator - (Vector2 const& left, Vector2 const& right)
{
	return Vector2(left.x - right.x, left.y - right.y);
}

Vector2 operator - (Vector2 const& left)
{
	return Vector2(-left.x, -left.y);
}

Vector2 operator * ( float left, Vector2 const &right )
{
	return Vector2( left * right.x, left * right.y );
}

constexpr float calculate_landing_radius()
{
	// In worst case, aircraft is located in opposite direction from decreasing the speed
	// So we need to rotate on 180 degrees
	constexpr float ROTATION_TIME = M_PI / params::aircraft::ANGULAR_SPEED;
	constexpr float SLOWDOWN_TIME = (params::aircraft::LINEAR_SPEED - params::aircraft::LANDING_SPEED) / params::aircraft::LINEAR_ACCELERATION;

	// During rotation, aircraft doesn't change its velocity (or it's needed to turn off lateral speed)
	// So, in worst case aircaft flight with LINEAR_SPEED for ROTATION_TIME
	constexpr float ROTATION_TRAVEL = ROTATION_TIME * params::aircraft::LINEAR_SPEED;

	// Here the aircraft slows down its speed, so we need to calculate simple integral
	constexpr float SLOWDOWN_TRAVEL = (params::aircraft::LINEAR_SPEED - params::aircraft::LANDING_SPEED) * SLOWDOWN_TIME / 2.f;

	return ROTATION_TRAVEL + SLOWDOWN_TRAVEL;
}

class Aircraft
{
public:
	Aircraft(Vector2 position = Vector2(0.f, 0.f), float angle = 0.f);

	/*
	 * Here we can avoid last 2 parameters by adding it to ship state (i.e. add them to members and create getters)
	 * But I think, that it's not a good idea in this case to increase state
	 * 
	 * We use access to the ship via game::ship. A coupling between Aircraft and Ship is quite bad
	 * It can be solved using interface, which add access to position, rotation (and delta_position, delta_rotation)
	 * In this case, ship should implenets this interface
	 * We can construct Aircraft by passing this interface via constructor parameter
	 * But in case, where we have rule, that there is only one ship, I think its overkill
	 */
	bool update(float dt, float ship_delta_rotation, Vector2 ship_delta_velocity);

private:
	void update_linear_velocity(float dt);

	static constexpr float LANDING_RADIUS = calculate_landing_radius();

	float calculate_rotation(const Vector2 & destination, float dt) const;

	Vector2 calculate_corrected_destination() const;
	Vector2 correct_closing_to_target(const Vector2& destination) const;
	Vector2 calculate_destination() const;
	Vector2 calculate_landing_destination() const;
	Vector2 calculate_target_destination() const;

	static Vector2 get_intersection(
		const Vector2& position_1, const Vector2& vector_1,
		const Vector2& position_2, const Vector2& vector_2);

	std::unique_ptr<scene::Mesh, decltype(&scene::destroyMesh)> m_mesh;
	Vector2 m_position;
	float m_angle;
	Vector2 m_linear_velocity;

	float m_live_time = 0.f;
};

//-------------------------------------------------------
//	Simple ship logic
//-------------------------------------------------------

class Ship
{
public:
	Ship();

	void init();
	void deinit();
	void update(float dt);
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseClicked(Vector2 worldPosition, bool isLeftButton);

	const Vector2 & get_position() { return position; }
	float get_angle() { return angle; }

private:
	scene::Mesh* mesh;
	Vector2 position;
	float angle;

	bool input[game::KEY_COUNT];

	std::vector<Aircraft> m_aircrafts;
	std::vector<float> m_aircraft_refill_timers;

	float m_live_time = 0.f;
};

//-------------------------------------------------------
//	game public interface
//-------------------------------------------------------

namespace game
{
	Ship ship;
	Vector2 s_goal_position;


	void init()
	{
		ship.init();
	}


	void deinit()
	{
		ship.deinit();
	}


	void update(float dt)
	{
		ship.update(dt);
	}


	void keyPressed(int key)
	{
		ship.keyPressed(key);
	}


	void keyReleased(int key)
	{
		ship.keyReleased(key);
	}


	void mouseClicked(float x, float y, bool isLeftButton)
	{
		Vector2 worldPosition(x, y);
		scene::screenToWorld(&worldPosition.x, &worldPosition.y);
		ship.mouseClicked(worldPosition, isLeftButton);
	}
}

Aircraft::Aircraft(Vector2 position, float angle)
	: m_mesh(scene::createAircraftMesh(), &scene::destroyMesh)
	, m_position(position)
	, m_angle(angle)
{
}

bool Aircraft::update(float dt, float ship_delta_rotation, Vector2 ship_delta_velocity)
{
	if (m_live_time >= params::aircraft::LIVE_TIME) {
		// delete airplane if it close enought for ship and its live time exceeds
		if ((game::ship.get_position() - m_position).get_length() <= params::ship::SIZE) {
			return false;
		}
	}

	// At the beginning of the flight we should to block own aircraft rotation to run to the runway
	if (m_live_time < params::aircraft::TAKEOFF_TIME) {
		m_linear_velocity = m_linear_velocity + ship_delta_velocity;

		m_angle = game::ship.get_angle();
		m_position = (m_position - game::ship.get_position()).get_rotated(ship_delta_rotation) + game::ship.get_position();
	}
	else {
		Vector2 destination = calculate_corrected_destination();
		float delta_rotation = calculate_rotation(destination, dt);

		m_angle = m_angle + delta_rotation;
	}

	update_linear_velocity(dt);

	m_position = m_position + dt * m_linear_velocity;
	scene::placeMesh(m_mesh.get(), m_position.x, m_position.y, m_angle);

	m_live_time += dt;
	return true;
}

void Aircraft::update_linear_velocity(float dt)
{
	Vector2 normalized_velocity(std::cos(m_angle), std::sin(m_angle));
	m_linear_velocity = m_linear_velocity + params::aircraft::LINEAR_ACCELERATION * dt * normalized_velocity;

	if (m_linear_velocity.get_length() > params::aircraft::LINEAR_SPEED) {
		m_linear_velocity = params::aircraft::LINEAR_SPEED * m_linear_velocity.get_normalized();
	}
}

float Aircraft::calculate_rotation(const Vector2 & destination, float dt) const
{
	Vector2 normalized_velocity(std::cos(m_angle), std::sin(m_angle));
	float target_angle = Vector2::angle_rad(destination, normalized_velocity);
	if (target_angle > 0) {
		float rotation = params::aircraft::ANGULAR_SPEED * dt;

		return std::min(rotation, target_angle);
	}
	else {
		float rotation = -params::aircraft::ANGULAR_SPEED * dt;

		return std::max(rotation, target_angle);
	}
}

Vector2 Aircraft::calculate_corrected_destination() const
{
	Vector2 destination = calculate_destination();
	destination = correct_closing_to_target(destination);

	/*
	 * We need to elimenate non-helpfull velocity
	 * We also want to achieve maximum speed for the destination vector
	 */
	destination = params::aircraft::LINEAR_SPEED * destination.get_normalized() - m_linear_velocity;

	return destination;
}

Vector2 Aircraft::calculate_destination() const
{
	return (m_live_time >= params::aircraft::LIVE_TIME) ? calculate_landing_destination() : calculate_target_destination();
}

Vector2 Aircraft::calculate_landing_destination() const
{
	Vector2 ship_forward = Vector2(1.f, 0.f).get_rotated(game::ship.get_angle());
	Vector2 ship_forward_normal = ship_forward.get_rotated(M_PI / 2.f);
	Vector2 intersection = get_intersection(game::ship.get_position(), ship_forward, m_position, ship_forward_normal);

	float length_to_intersection = (intersection - m_position).get_length();

	if (length_to_intersection > 0.01f) {
		if (length_to_intersection > LANDING_RADIUS) {
			// Step 1 - go close to the ship forward vector normal (TARGET_RADIUS based component is used to get smooth rotation)
			return intersection + LANDING_RADIUS * (m_position - intersection).get_normalized() - m_position;
		}
		else {
			// Step 2 - rotate to be in the ship forward vector
			return intersection + LANDING_RADIUS * (game::ship.get_position() - intersection).get_normalized() - m_position;
		}
	}
	else {
		// Step 3 - got to the ship
		return game::ship.get_position() - m_position;
	}
}

Vector2 Aircraft::get_intersection(
	const Vector2& position_1, const Vector2& vector_1,
	const Vector2& position_2, const Vector2& vector_2)
{
	/*
	 * here we need to solve a system of equations
	 * position_1.x + vector_1.x * n = position_2.x + vector_2.x * k
	 * position_1.y + vector_1.y * n = position_2.y + vector_2.y * k
	 */

	float k = (position_2.y - position_1.y - ((vector_1.y / vector_1.x) * (position_2.x - position_1.x))) /
		((vector_1.y * vector_2.x / vector_1.x) - vector_2.y);

	return Vector2(position_2.x + vector_2.x * k, position_2.y + vector_2.y * k);
}

Vector2 Aircraft::calculate_target_destination() const
{
	// Vector to goal
	Vector2 target_vector = game::s_goal_position - m_position;

	/*
	 * As we want to move around the target, we can move to the normal of the target vector
	 * Target vector will be recalculated on each frame, so normal will be also recalculated
	 * and aircraft will tries to moving to the circle
	 */
	Vector2 goal_position = game::s_goal_position + params::aircraft::TARGET_RADIUS * target_vector.get_rotated(M_PI / 2.f).get_normalized();

	return goal_position - m_position;
}

Vector2 Aircraft::correct_closing_to_target(const Vector2& destination) const
{
	if (destination.get_length() <= LANDING_RADIUS) {
		// get projection of current velcity to target vector
		float projection = (Vector2::dot(m_linear_velocity, destination) * m_linear_velocity).get_length();

		if (projection > params::aircraft::LANDING_SPEED) {
			return -destination;
		}
	}
	return destination;
}

Ship::Ship() :
	mesh( nullptr )
{
	// Reserve space for 5 Aircrafts to avoid extra allocations
	m_aircrafts.reserve(5);
	m_aircraft_refill_timers.reserve(5);
}

void Ship::init()
{
	assert( !mesh );
	mesh = scene::createShipMesh();
	position = Vector2( 0.f, 0.f );
	angle = 0.f;
	for ( bool &key : input )
		key = false;
}

void Ship::deinit()
{
	scene::destroyMesh( mesh );
	mesh = nullptr;
}

void Ship::update( float dt )
{
	float linearSpeed = 0.f;
	float angularSpeed = 0.f;

	if ( input[ game::KEY_FORWARD ] )
	{
		linearSpeed = params::ship::LINEAR_SPEED;
	}
	else if ( input[ game::KEY_BACKWARD ] )
	{
		linearSpeed = -params::ship::LINEAR_SPEED;
	}

	if ( input[ game::KEY_LEFT ] && linearSpeed != 0.f )
	{
		angularSpeed = params::ship::ANGULAR_SPEED;
	}
	else if ( input[ game::KEY_RIGHT ] && linearSpeed != 0.f )
	{
		angularSpeed = -params::ship::ANGULAR_SPEED;
	}

	float ship_rotation = angularSpeed * dt;
	angle = angle + ship_rotation;

	Vector2 ship_velocity = linearSpeed * dt * Vector2(std::cos(angle), std::sin(angle));
	position = position + ship_velocity;
	scene::placeMesh( mesh, position.x, position.y, angle );

	for (auto it = m_aircraft_refill_timers.begin(); it != m_aircraft_refill_timers.end();) {
		if (m_live_time >= *it + params::ship::REFILL_TIME) {
			it = m_aircraft_refill_timers.erase(it);
		}
		else {
			++it;
		}
	}

	for (auto it = m_aircrafts.begin(); it != m_aircrafts.end();) {
		if (!it->update(dt, ship_rotation, ship_velocity)) {
			it = m_aircrafts.erase(it);
			m_aircraft_refill_timers.emplace_back(m_live_time);
		}
		else {
			++it;
		}
	}
	m_live_time += dt;
}

void Ship::keyPressed( int key )
{
	assert( key >= 0 && key < game::KEY_COUNT );
	input[ key ] = true;
}

void Ship::keyReleased( int key )
{
	assert( key >= 0 && key < game::KEY_COUNT );
	input[ key ] = false;
}

void Ship::mouseClicked(Vector2 worldPosition, bool isLeftButton)
{
	// TODO: placeholder, remove it and implement aircarfts logic
	if (isLeftButton)
	{
		scene::placeGoalMarker(worldPosition.x, worldPosition.y);
		game::s_goal_position = worldPosition;
	}
	else
	{
		if (m_aircrafts.size() + m_aircraft_refill_timers.size() < 5) {
			m_aircrafts.emplace_back(position, angle);
		}
	}
}
