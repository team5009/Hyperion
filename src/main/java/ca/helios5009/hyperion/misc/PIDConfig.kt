package ca.helios5009.hyperion.misc

open class PIDConfig {

	open var GainSpeed: Double = 0.0;

	@JvmField var AccelerationLimit: Double = 0.0;
	@JvmField var Tolerance: Double = 0.0;
	@JvmField var Deadband: Double = 0.0;
}