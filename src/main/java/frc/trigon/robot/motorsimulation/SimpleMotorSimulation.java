package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimpleMotorSimulation extends MotorSimulation {
    private final DCMotorSim motorSimulation;

    public SimpleMotorSimulation(DCMotor motor, double gearRatio, double momentOfInertia) {
        motorSimulation = new DCMotorSim(motor, gearRatio, momentOfInertia);
    }

    @Override
    double calculateFeedforward(double ks, double kg, double kv, double ka) {
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        return feedforward.calculate(getVelocityRevolutionsPerSecond());
    }

    @Override
    double getPositionRevolutions() {
        return Units.radiansToRotations(motorSimulation.getAngularPositionRad());
    }

    @Override
    double getVelocityRevolutionsPerSecond() {
        return Units.radiansToRotations(motorSimulation.getAngularVelocityRadPerSec());
    }

    @Override
    double getCurrent() {
        return motorSimulation.getCurrentDrawAmps();
    }

    @Override
    void setInputVoltage(double voltage) {
        motorSimulation.setInputVoltage(voltage);
    }
}
