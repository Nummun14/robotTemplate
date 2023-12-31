package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.utilities.Conversions;

public class SingleJointedArmSimulation extends MotorSimulation {
    private final SingleJointedArmSim armSim;

    public SingleJointedArmSimulation(DCMotor motor, double gearRatio, double momentOfInertia, double retractedArmLengthMeters, double minimumAngleRadians, double maximumAngleRadians, boolean simulateGravity) {
        armSim = new SingleJointedArmSim(motor, gearRatio, momentOfInertia, retractedArmLengthMeters, minimumAngleRadians, maximumAngleRadians, simulateGravity, minimumAngleRadians);
    }

    @Override
    double getPositionRevolutions() {
        double positionDegrees = Units.radiansToDegrees(armSim.getAngleRads());
        return Conversions.degreesToRevolutions(positionDegrees);
    }

    @Override
    double getVelocityRevolutionsPerSecond() {
        double velocityDegreesPerSecond = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
        return Conversions.degreesToRevolutions(velocityDegreesPerSecond);
    }

    @Override
    double getCurrent() {
        return armSim.getCurrentDrawAmps();
    }

    @Override
    void setInputVoltage(double voltage) {
        armSim.setInputVoltage(voltage);
    }
}
