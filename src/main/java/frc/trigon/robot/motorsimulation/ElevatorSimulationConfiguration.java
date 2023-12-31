package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorSimulationConfiguration {
    private ElevatorFeedforward feedforward;
    private TrapezoidProfile.Constraints constraints;
    private double
            kp,
            ki,
            kd;

    public ElevatorSimulationConfiguration() {
    }

    public void setPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setFeedforward(double ks,double kg, double kv, double ka) {
        this.feedforward = new ElevatorFeedforward(ks, kg, kv, ka);
    }

    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    }

    public ProfiledPIDController getProfiledPIDController() {
        return new ProfiledPIDController(kp, ki, kd, constraints);
    }

    public ElevatorFeedforward getFeedforward() {
        return feedforward;
    }
}
