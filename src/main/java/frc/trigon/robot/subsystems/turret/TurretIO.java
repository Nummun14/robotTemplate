package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.turret.kablamaturret.KablamaTurretIO;
import frc.trigon.robot.subsystems.turret.simulationturret.SimulationTurretIO;
import org.littletonrobotics.junction.AutoLog;

public class TurretIO {
    static TurretIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new TurretIO();
        if (RobotConstants.ROBOT_TYPE == RobotConstants.RobotType.KABLAMA)
            return new KablamaTurretIO();
        return new SimulationTurretIO();
    }

    protected void updateInputs(TurretInputsAutoLogged inputs) {
    }

    protected void setTargetAngle(Rotation2d targetAngle) {
    }

    protected void stop() {
    }

    @AutoLog
    protected static class TurretInputs {
        public double motorPositionDegrees = 0;
        public double motorVelocityDegreesPerSecond = 0;
        public double motorVoltage = 0;
        public double profiledTargetPositionDegrees = 0;
    }
}