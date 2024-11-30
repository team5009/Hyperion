package ca.helios5009.hyperion.core

import kotlin.math.abs
import kotlin.math.min

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
	var kP: Double,
	var kI: Double,
	var kD: Double,
	var kF: Double
) {
	var targetPoint: Double = 0.0;

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
	 * @see atSetPoint
	 */
	fun setTolerance(positionTolerance: Double, velocityTolerance: Double) {
		this.positionTolerance = positionTolerance;
		this.velocityTolerance = velocityTolerance;
	}

	/**
	 * Returns true if the error is within the percentage of the total input range, determined by [setTolerance](setTolerance).
	 * @see setTolerance
	 * @return True if the error is within the tolerance.
	 */
	fun isAtTarget(): Boolean {
		return abs(positionError) < positionTolerance && abs(velocityError) < velocityTolerance;
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
	 * @return The output of the controller.
	 */
	fun getCoefficients(): List<Double> {
		return listOf(kP, kI, kD, kF);
	}

	/**
	 * Calculates the control value, u(t).
	 *
	 * @param pv The current measurement of the process variable.
	 * @return the value produced by u(t).
	 */
	fun calculate(pv: Double) : Double {
		prevPositionError = positionError;
		val currentTime = System.nanoTime() / 1e9;

		if (lastTime.equals(0.0))
			lastTime = currentTime;

		period = currentTime - lastTime;
		lastTime = currentTime;

		positionError = if (measuredValue.equals(pv)) {
			targetPoint - measuredValue;
		} else {
			measuredValue = pv;
			targetPoint - pv;
		}

		velocityError = if (abs(period) > 1e-6) {
			(positionError - prevPositionError) / period;
		} else {
			0.0;
		}

		/*
	        if total error is the integral from 0 to t of e(t')dt', and
	        e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
		totalError += period * (targetPoint - measuredValue);
		totalError = if (totalError < minIntegral) {
			minIntegral
		} else {
			min(maxIntegral, totalError)
		}

		return kP * positionError + kI * totalError + kD * velocityError + kF * targetPoint;
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
	fun calculate() : Double {
		return calculate(measuredValue);
	}

	fun setPIDF(kP: Double, kI: Double, kD: Double, kF: Double) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
	}

	fun setPID(kP: Double, kI: Double, kD: Double): PIDFController  {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		return this;
	}

	fun setP(kP: Double): PIDFController {
		this.kP = kP;
		return this;
	}

	fun setI(kI: Double): PIDFController {
		this.kI = kI;
		return this;
	}

	fun setD(kD: Double): PIDFController {
		this.kD = kD;
		return this;
	}

	fun setF(kF: Double): PIDFController {
		this.kF = kF;
		return this;
	}

	fun getPeriod(): Double {
		return period;
	}




}