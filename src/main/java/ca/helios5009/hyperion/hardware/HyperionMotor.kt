package ca.helios5009.hyperion.hardware

import com.qualcomm.robotcore.hardware.DcMotorEx

/**
 * HyperionMotor is a class that allows for the creation of motors for the robot to use.
 *
 * What makes this class different is it caches the power of the motor. This is to prevent
 * setting the power of the motor to the same value.
 * Every time the power is changed, the call from the control hub to the motor takes time.
 * Caching makes it so the motor doesn't have to be set to the same power.
 *
 * @param motor The DcMotorEx that the HyperionMotor is controlling.
 *
 * @author Joshua
 * @see DcMotorEx
 */
class HyperionMotor(val motor: DcMotorEx) {
	private var power = 0.0
	fun setPower(power: Double) {
		if (power != this.power) {
			this.power = power
			motor.power = power
		}
	}

	fun stop() {
		setPower(0.0)
	}
}