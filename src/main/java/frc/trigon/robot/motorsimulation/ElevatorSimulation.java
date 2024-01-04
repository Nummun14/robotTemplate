package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.trigon.robot.utilities.Conversions;

public class ElevatorSimulation extends MotorSimulation {
    private final ElevatorSim elevatorSimulation;
    private final double diameterMeters;

    public ElevatorSimulation(DCMotor motor, double gearRatio, double mass, double drumRadiusMeters, double retractedArmLengthMeters, double maximumAngleRadians, boolean simulateGravity) {
        this.diameterMeters = drumRadiusMeters + drumRadiusMeters;
        elevatorSimulation = new ElevatorSim(motor, gearRatio, mass, drumRadiusMeters, retractedArmLengthMeters, maximumAngleRadians, simulateGravity, retractedArmLengthMeters);
    }
    @Override
    double calculateFeedforward(double ks, double kg, double kv, double ka, double targetVelocity, double targetPosition) {
        return ks * Math.signum(targetVelocity) + kg + kv * targetVelocity + ka * 0;
    }

    @Override
    double getPositionRevolutions() {
        return Conversions.distanceToRevolutions(elevatorSimulation.getPositionMeters(), diameterMeters);
    }

    @Override
    double getVelocityRevolutionsPerSecond() {
        return Conversions.distanceToRevolutions(elevatorSimulation.getVelocityMetersPerSecond(), diameterMeters);
    }

    @Override
    double getCurrent() {
        return elevatorSimulation.getCurrentDrawAmps();
    }

    @Override
    void setInputVoltage(double voltage) {
        elevatorSimulation.setInputVoltage(voltage);
    }
}
