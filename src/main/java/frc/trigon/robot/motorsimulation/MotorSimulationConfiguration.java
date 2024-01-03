package frc.trigon.robot.motorsimulation;

public class MotorSimulationConfiguration {
    public double conversionFactor = 1;
    public double
            kP = 1 * conversionFactor,
            kI = 0 * conversionFactor,
            kD = 0 * conversionFactor;
    public double
            kS = 0 * conversionFactor,
            kG = 9.8 * conversionFactor,
            kV = 0 * conversionFactor,
            kA = 0 * conversionFactor;
    public double voltageCompensationSaturation = 12 * conversionFactor;
    public double
            maxVelocity = 100 * conversionFactor,
            maxAcceleration = 80 * conversionFactor;
    public double
            minimumContinuousOutput = 6 * conversionFactor,
            maximumContinuousOutput = 12 * conversionFactor;
}
