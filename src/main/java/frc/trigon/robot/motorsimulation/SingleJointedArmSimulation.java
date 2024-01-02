package frc.trigon.robot.motorsimulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.utilities.Conversions;

public class SingleJointedArmSimulation extends MotorSimulation {
    private final SingleJointedArmSim armSimulation;

    public SingleJointedArmSimulation(DCMotor motor, double gearRatio, double retractedArmLengthMeters, double minimumAngleRadians, double maximumAngleRadians, boolean simulateGravity, double armMass) {
        armSimulation = new SingleJointedArmSim(motor, gearRatio, SingleJointedArmSim.estimateMOI(retractedArmLengthMeters, armMass), retractedArmLengthMeters, minimumAngleRadians, maximumAngleRadians, simulateGravity, minimumAngleRadians);
    }

    @Override
    double getPositionRevolutions() {
        return Units.radiansToRotations(armSimulation.getAngleRads());
    }

    @Override
    double getVelocityRevolutionsPerSecond() {
        return Units.radiansToRotations(armSimulation.getVelocityRadPerSec());
    }

    @Override
    double getCurrent() {
        return armSimulation.getCurrentDrawAmps();
    }

    @Override
    void setInputVoltage(double voltage) {
        armSimulation.setInputVoltage(voltage);
    }
}
