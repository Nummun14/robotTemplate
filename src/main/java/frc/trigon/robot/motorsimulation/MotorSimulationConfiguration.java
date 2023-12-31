package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class MotorSimulationConfiguration {
    public double
            kP,
            kI,
            kD;
    public double
            kS,
            kG,
            kV,
            kA;
    public TrapezoidProfile.Constraints constraints;

    public MotorSimulationConfiguration() {
    }
}
