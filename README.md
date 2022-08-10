# Simple aircraft movement model.

# Movement behaviour
In the nutshell I assume, that aircraft always have an acceleration. In other case it will fall down.
So, everytime we have a force, which change our velocity.

Using this assumption, we understand that we can correct own speed by rotating to specific angle.
Aircraft force will continously change out velocity to achive correct velocity.

To get correct angle I use these findings:
1) We need to move to the target.
2) It's nice to move to the target with maximum velocity.
3) We need to eleminate "parasite" velocity - the velocity, which move aircraft around the target.

So, the final formula is:

LINEAR_SPEED * Target_vec_norm - Velocity, where:

LINEAR_SPEED - maximum possible aircraft speed (float),

Target_vec_norm - normalized vector to the target,

Velocity - current velocity vector.

When aircraft approaching to the target and aircraft spped is too fast, I change Target_vec_norm to the opposite to prevent misses.
You can set LANDING_SPEED = LINEAR_SPEED to disable this behaviour.

# Targets
To flight around the target aircraft always tries to go to the normal between aircraft and goal (normalized, multiplied to the rotation radius).

# Landing
Landing to the ship is performed in 3 steps:
1) Flight close to the ship forward vector.

I calculate normal to this vector and solve simple system of equations to get intersection between normal vector, which contains aicraft position and ship forward vector.

Aircraft will be distanceeated from intersection point (by normal) to some value to achive smoother movement in the next steps.

2) Flight in forward of ship forward vector.

Here I also use intersection point and some value (but in this case by forward vector) to achive moving aircraft to the arc.

3) Go to the ship

When aircraft goes close to the ship it also tries to slow down its speed to land more smoothly.
