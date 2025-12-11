package ca.helios5009.hyperion.core

import ca.helios5009.hyperion.misc.epsilonEquals
import kotlin.math.sign

object Kinematics {
    fun calculateMotorFeedforward(
        vels: List<Double>,
        accels: List<Double>,
        kV: Double,
        kA: Double,
        kStatic: Double
    ) =
        vels.zip(accels)
            .map { (vel, accel) -> calculateMotorFeedforward(vel, accel, kV, kA, kStatic) }

    /**
     * Computes the motor feedforward (i.e., open loop power) for the given set of coefficients.
     */
    fun calculateMotorFeedforward(vel: Double, accel: Double, kV: Double, kA: Double, kStatic: Double): Double {
        val basePower = vel * kV + accel * kA
        return if (basePower epsilonEquals 0.0) {
            0.0
        } else {
            basePower + sign(basePower) * kStatic
        }
    }
}