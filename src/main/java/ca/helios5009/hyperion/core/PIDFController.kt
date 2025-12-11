package ca.helios5009.hyperion.core

import ca.helios5009.hyperion.misc.epsilonEquals
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign

/**
 * A PIDF controller class that can be used to control a system.
 * Code adapted from FTC Lib's PIDFController class.
 * @param kP The proportional gain of the controller.
 * @param kI The integral gain of the controller.
 * @param kD The derivative gain of the controller.
 * @param kF The feedforward gain of the controller.
 *
 */
class PIDFController(
	private val pid: PIDCoefficients,
	private val kV: Double = 0.0,
	private val kA: Double = 0.0,
	private val kF: (Double, Double?) -> Double = { _, _ -> 0.0 },
	private val kStatic: Double = 0.0,
) {
	var targetPoint: Double = 0.0;
	var targetVelocity: Double = 0.0;
	var targetAcceleration: Double = 0.0;

	private var measuredValue: Double = 0.0;
	private var minIntegral: Double = -1.0;
	private var maxIntegral: Double = 1.0;

	/**
	 * The error in the position of the system.
	 */
	var positionError: Double = 0.0

	/**
	 * The error in the velocity of the system.
	 */
	var velocityError: Double = 0.0


	private var totalError: Double = 0.0;
	private var prevPositionError: Double = 0.0;

	private var positionTolerance = 0.05;
	private var velocityTolerance = Double.POSITIVE_INFINITY;

	private var lastTime: Double = 0.0;
	private var period: Double = 0.0;

	private var circular = false;

	/**
	 * Returns true if the error is within the percentage of the total input range, determined by [setTolerance](setTolerance).
	 * @see setTolerance
	 * @return True if the error is within the tolerance.
	 */
	val isAtTarget get() = abs(positionError) < positionTolerance && abs(velocityError) < velocityTolerance;

	/**
	 * Returns the tolerances of the controller.
	 */
	val tolerances get() = Pair(positionTolerance, velocityTolerance);

	/**
	 * Returns the coefficients of the controller.
	 */
	val coefficients get() = pid;

	fun reset() {
		totalError = 0.0;
		prevPositionError = 0.0;
		lastTime = 0.0;
	}

	/**
	 * Sets the error which is considered tolerable for use with {@atSetPoint}.
	 *
	 * @param positionTolerance Position error which is tolerable.
	 * @param velocityTolerance Velocity error which is tolerable.
	 * @see isAtTarget
	 */
	fun setTolerance(positionTolerance: Double, velocityTolerance: Double) {
		this.positionTolerance = positionTolerance;
		this.velocityTolerance = velocityTolerance;
	}

	/**
	 * Sets the target point for the controller.
	 */
	fun setTarget(target: Double) {
		measuredValue = target;
		positionError = targetPoint - measuredValue;
		velocityError = (positionError - prevPositionError) / period;
	}

	/**
	 * Enables the controller to wrap around the angle.
	 */
	fun setAngleWrapAround() {
		circular = true;
	}

	private fun angleWrap(err: Double): Double {
		var error = err;
		while (error > Math.PI) error -= 2 * Math.PI
		while (error <= -Math.PI) error += 2 * Math.PI
		return error;
	}


	private fun getPositionError(measuredPoint: Double) : Double {
		return if (circular) {
			angleWrap(targetPoint - measuredPoint)
		} else {targetPoint - measuredPoint}
	}
	/**
	 * Calculates the control value, u(t).
	 *
	 * @param pv The current measurement of the process variable.
	 * @return the value produced by u(t).
	 */
	fun calculate(pv: Double) : Double {
		if (!measuredValue.equals(pv)) {
			measuredValue = pv;
		}
		return directCalculate(targetPoint - measuredValue)
	}

	fun directCalculate(measuredPoint: Double, measuredVelocity: Double? = null, t:Telemetry? = null): Double {
		prevPositionError = positionError;
		val currentTime : Double = (System.nanoTime() / 1e9)
		if (lastTime.equals(0.0)) lastTime = currentTime

		period = currentTime - lastTime
		lastTime = currentTime
		positionError = getPositionError(measuredPoint)
		if (t != null){
			t.addData("break point 1", positionError)
			t.addData("break point 2", lastTime)
		}
		velocityError = if (abs(period) > 1e-6) {
			(positionError - prevPositionError) / period;
		} else {
			0.0;
		}

		totalError += period * (measuredPoint);
		totalError = if (totalError < minIntegral) {
			minIntegral
		} else {
			min(maxIntegral, totalError)
		}
		if (t != null){
			t.addData("break point 3", totalError)
			t.addData("break point 4", measuredPoint)
		}
		val baseOutput = pid.kP * positionError + pid.kI * totalError +
			pid.kD * (measuredVelocity?.let { targetVelocity - it } ?: velocityError) +
			kV * targetVelocity + kA * targetAcceleration + kF(targetPoint, measuredValue)

		return if (baseOutput epsilonEquals 0.0) 0.0 else baseOutput + sign(baseOutput) * kStatic
	}

	/**
	 * Calculates the next output of the PIDF controller.
	 *
	 * @param pv The given measured value.
	 * @param sp The given set point.
	 * @return the next output using the given measured value via
	 * @see calculate(Double)
	 */
	fun calculate(pv: Double, sp: Double) : Double {
		// set the set point to the provided value
		targetPoint = sp;
		return calculate(pv);
	}

	/**
	 * Calculates the next output of the PIDF controller.
	 *
	 * @return the next output using the current measured value via
	 * @see calculate(Double)
	 */
	fun calculate() = calculate(measuredValue);

	fun getPeriod() = period;
}