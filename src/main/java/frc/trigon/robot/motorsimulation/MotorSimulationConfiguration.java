package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class MotorSimulationConfiguration {
    private SimpleMotorFeedforward feedforward;
    private TrapezoidProfile.Constraints constraints;
    private double
            kp,
            ki,
            kd;

    public MotorSimulationConfiguration() {
    }

    public void setPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setFeedforward(double ks, double kv, double ka) {
        this.feedforward = new SimpleMotorFeedforward(ks, kv, ka);
    }

    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    }

    public ProfiledPIDController getProfiledPIDController() {
        return new ProfiledPIDController(kp, ki, kd, constraints);
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }
}
