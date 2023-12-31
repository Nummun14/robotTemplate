package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorSimulation {
    private final double metersPerRevolutions = 4028;
    private final ElevatorSim elevatorSim;
    private ProfiledPIDController pidController;
    private ElevatorFeedforward feedforward;
    private double voltage = 0;

    public ElevatorSimulation(DCMotor motor, double gearRatio, double mass, double drumRadiusMeters, double retractedArmLengthMeters, double maximumAngleRadians, boolean simulateGravity) {
        elevatorSim = new ElevatorSim(motor, gearRatio, mass, drumRadiusMeters, retractedArmLengthMeters, maximumAngleRadians, simulateGravity, retractedArmLengthMeters);
    }

    public void applyConfiguration(ElevatorSimulationConfiguration config) {
        this.pidController = config.getProfiledPIDController();
        this.feedforward = config.getFeedforward();
    }

    public void stop() {
        voltageRequest(0);
    }

    public void voltageRequest(double voltage) {
        this.voltage = voltage;
        elevatorSim.setInputVoltage(voltage);
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
        return elevatorSim.getPositionMeters() * metersPerRevolutions;
    }

    public double getVelocityRevolutionsPerSecond() {
        return elevatorSim.getVelocityMetersPerSecond() * metersPerRevolutions;
    }

    public double getCurrent() {
        return elevatorSim.getCurrentDrawAmps();
    }
}
