package frc.trigon.robot.subsystems.turret.simulationturret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.motorsimulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.turret.TurretIO;
import frc.trigon.robot.subsystems.turret.TurretInputsAutoLogged;

public class SimulationTurretIO extends TurretIO {
    private final SimpleMotorSimulation motor = SimulationTurretConstants.MOTOR;
    private final MotionMagicVoltage positionVoltageRequest = new MotionMagicVoltage(0);
    private double voltage = 0;

    @Override
    protected void updateInputs(TurretInputsAutoLogged inputs) {

        inputs.motorPositionDegrees = Units.rotationsToDegrees(motor.getPositionRevolutions());
        inputs.motorVelocityDegreesPerSecond = Units.rotationsToDegrees(motor.getVelocityRevolutionsPerSecond());
        inputs.motorVoltage = voltage;
        inputs.profiledTargetPositionDegrees = Units.rotationsToDegrees(motor.getProfiledTargetPositionRevolutions());
    }

    @Override
    protected void setTargetAngle(Rotation2d targetAngle) {
        motor.setControl(positionVoltageRequest.withPosition(targetAngle.getRotations()));
    }

    @Override
    protected void stop() {
        motor.stop();
    }
}
