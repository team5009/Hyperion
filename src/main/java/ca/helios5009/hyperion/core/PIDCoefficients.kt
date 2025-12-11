package ca.helios5009.hyperion.core

data class PIDCoefficients(
    @JvmField var kP: Double = 0.0,
    @JvmField var kI: Double = 0.0,
    @JvmField var kD: Double = 0.0,
    @JvmField var tolerance: Double = 0.0,
    @JvmField var velTolerance: Double = Double.POSITIVE_INFINITY,
)