# Efficient 2D Movement Prediction for Missile Launch

## Description

This C++ source file provides an efficient implementation of 2D movement prediction to determine the optimal launch position for a "missile" to hit a moving target. The algorithm takes into consideration the target's trajectory, launch delay, the possibility of the enemy moving along a list of points instead of a straight line, and the scenario when the missile can't hit the target (e.g., when the target is moving away faster than the missile speed) all using a simple math formula.

## Features

- Predicts target's position based on its movement trajectory
- Accounts for launch delay to ensure accurate targeting
- Handles non-linear movement patterns by considering a list of points
- Avoids launching a missile when the target is impossible to hit
- Efficient and simple in just one formula

## Math Behind the Prediction
Raises the equation of a circle whose radius is expressed according to time (this formula represents all the possible points where the missile could be launched at) and then express the enemy position according to time (where the enemy would be at "x" time). Express the circle radius as (time - start_time) * speed to take into consideration missile delay and the enemy's next start position's time (when enemy is moving to a list of points). Use the distance formula from a point to a circle and replace the previous formulas in it. This quadratic equation should give you the time when the missile will collide with the target. Given this information, you can calculate where the target will be and you should get the position perfecly, it would even tell you if the enemy is impossible to be hit.
