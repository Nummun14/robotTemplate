package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorSimulation extends MotorSimulation {
    private final double metersPerRevolutions = 4028;
    private final ElevatorSim elevatorSim;

    public ElevatorSimulation(DCMotor motor, double gearRatio, double mass, double drumRadiusMeters, double retractedArmLengthMeters, double maximumAngleRadians, boolean simulateGravity) {
        elevatorSim = new ElevatorSim(motor, gearRatio, mass, drumRadiusMeters, retractedArmLengthMeters, maximumAngleRadians, simulateGravity, retractedArmLengthMeters);
    }

    @Override
    double getPositionRevolutions() {
        return elevatorSim.getPositionMeters() * metersPerRevolutions;
    }

    @Override
    double getVelocityRevolutionsPerSecond() {
        return elevatorSim.getVelocityMetersPerSecond() * metersPerRevolutions;
    }

    @Override
    double getCurrent() {
        return elevatorSim.getCurrentDrawAmps();
    }

    @Override
    void setInputVoltage(double voltage) {
        elevatorSim.setInputVoltage(voltage);
    }
}
