package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SingleJointedArmSimulationConfiguration {
    private ArmFeedforward feedforward;
    private TrapezoidProfile.Constraints constraints;
    private double
            kp,
            ki,
            kd;

    public SingleJointedArmSimulationConfiguration() {
    }

    public void setPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setFeedforward(double ks, double kg, double kv, double ka) {
        this.feedforward = new ArmFeedforward(ks, kg, kv, ka);
    }

    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    }

    public ProfiledPIDController getProfiledPIDController() {
        return new ProfiledPIDController(kp, ki, kd, constraints);
    }

    public ArmFeedforward getFeedforward() {
        return feedforward;
    }
}
