package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.utilities.Conversions;

public class SimpleMotorSimulation extends MotorSimulation {
    private final DCMotorSim motorSim;

    public SimpleMotorSimulation(DCMotor motor, double gearRatio, double momentOfInertia) {
        motorSim = new DCMotorSim(motor, gearRatio, momentOfInertia);
    }

    @Override
    double getPositionRevolutions() {
        double positionDegrees = Units.radiansToDegrees(motorSim.getAngularPositionRad());
        return Conversions.degreesToRevolutions(positionDegrees);
    }

    @Override
    double getVelocityRevolutionsPerSecond() {
        double velocityDegreesPerSecond = Units.radiansToDegrees(motorSim.getAngularVelocityRadPerSec());
        return Conversions.degreesToRevolutions(velocityDegreesPerSecond);
    }

    @Override
    double getCurrent() {
        return motorSim.getCurrentDrawAmps();
    }

    @Override
    void setInputVoltage(double voltage) {
        motorSim.setInputVoltage(voltage);
    }
}
