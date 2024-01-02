package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.trigon.robot.utilities.Conversions;

public class ElevatorSimulation extends MotorSimulation {
    private final ElevatorSim elevatorSimulation;
    private final double drumRadiusMeters;

    public ElevatorSimulation(DCMotor motor, double gearRatio, double mass, double drumRadiusMeters, double retractedArmLengthMeters, double maximumAngleRadians, boolean simulateGravity) {
        this.drumRadiusMeters = drumRadiusMeters;
        elevatorSimulation = new ElevatorSim(motor, gearRatio, mass, drumRadiusMeters, retractedArmLengthMeters, maximumAngleRadians, simulateGravity, retractedArmLengthMeters);
    }

    @Override
    double getPositionRevolutions() {
        return Conversions.distanceToRevolutions(elevatorSimulation.getPositionMeters(), drumRadiusMeters);
    }

    @Override
    double getVelocityRevolutionsPerSecond() {
        return Conversions.distanceToRevolutions(elevatorSimulation.getVelocityMetersPerSecond(), drumRadiusMeters);
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
