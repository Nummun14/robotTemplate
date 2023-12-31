package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.utilities.Conversions;

public class SingleJointedArmSimulation {
    private final SingleJointedArmSim armSim;
    private ProfiledPIDController pidController;
    private ArmFeedforward feedforward;
    private double voltage = 0;

    public SingleJointedArmSimulation(DCMotor motor, double gearRatio, double momentOfInertia, double retractedArmLengthMeters, double minimumAngleRadians, double maximumAngleRadians, boolean simulateGravity) {
        armSim = new SingleJointedArmSim(motor, gearRatio, momentOfInertia, retractedArmLengthMeters, minimumAngleRadians, maximumAngleRadians, simulateGravity, minimumAngleRadians);
    }

    public void applyConfiguration(SingleJointedArmSimulationConfiguration config) {
        this.pidController = config.getProfiledPIDController();
        this.feedforward = config.getFeedforward();
    }

    public void stop() {
        voltageRequest(0);
    }

    public void voltageRequest(double voltage) {
        this.voltage = voltage;
        armSim.setInputVoltage(voltage);
    }

    public void positionVoltageRequest(double targetPosition) {
        double pidOutput = pidController.calculate(getPositionRevolutions(), targetPosition);
        voltageRequest(pidOutput);
    }

    public void motionMagicRequest(double targetPosition) {
        double pidOutput = pidController.calculate(getPositionRevolutions(), targetPosition);
        double feedforwardOutput = feedforward.calculate(armSim.getAngleRads(), getVelocityRevolutionsPerSecond());
        double output = pidOutput + feedforwardOutput;
        voltageRequest(output);
    }

    public double getVoltage() {
        return voltage;
    }

    public double getPositionRevolutions() {
        double positionDegrees = Units.radiansToDegrees(armSim.getAngleRads());
        return Conversions.degreesToRevolutions(positionDegrees);
    }

    public double getVelocityRevolutionsPerSecond() {
        double velocityDegreesPerSecond = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
        return Conversions.degreesToRevolutions(velocityDegreesPerSecond);
    }

    public double getCurrent() {
        return armSim.getCurrentDrawAmps();
    }
}
