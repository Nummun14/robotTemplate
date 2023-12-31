package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.utilities.Conversions;

public class MotorSimulation {
    private final DCMotorSim motorSim;
    private ProfiledPIDController pidController;
    private SimpleMotorFeedforward feedforward;
    private double voltage = 0;

    public MotorSimulation(DCMotor motor, double gearRatio, double momentOfInertia) {
        motorSim = new DCMotorSim(motor, gearRatio, momentOfInertia);
    }

    public void applyConfiguration(MotorSimulationConfiguration config) {
        this.pidController = config.getProfiledPIDController();
        this.feedforward = config.getFeedforward();
    }

    public void stop() {
        voltageRequest(0);
    }

    public void voltageRequest(double voltage) {
        this.voltage = voltage;
        motorSim.setInputVoltage(voltage);
    }

    public void positionVoltageRequest(double targetPosition) {
        double pidOutput = pidController.calculate(getPositionRevolutions(), targetPosition);
        voltageRequest(pidOutput);
    }

    public void motionMagicRequest(double targetPosition) {
        double pidOutput = pidController.calculate(getPositionRevolutions(), targetPosition);
        double feedforwardOutput = feedforward.calculate(getVelocityRevolutionsPerSecond());
        double output = pidOutput + feedforwardOutput;
        voltageRequest(output);
    }

    public double getVoltage() {
        return voltage;
    }

    public double getPositionRevolutions() {
        double positionDegrees = Units.radiansToDegrees(motorSim.getAngularPositionRad());
        return Conversions.degreesToRevolutions(positionDegrees);
    }

    public double getVelocityRevolutionsPerSecond() {
        double velocityDegreesPerSecond = Units.radiansToDegrees(motorSim.getAngularVelocityRadPerSec());
        return Conversions.degreesToRevolutions(velocityDegreesPerSecond);
    }

    public double getCurrent() {
        return motorSim.getCurrentDrawAmps();
    }
}
