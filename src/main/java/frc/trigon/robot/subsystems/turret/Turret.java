package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final static Turret INSTANCE = new Turret();
    private final TurretIO turretIO = TurretIO.generateIO();
    private final TurretInputsAutoLogged turretInputs = new TurretInputsAutoLogged();

    public static Turret getInstance() {
        return INSTANCE;
    }

    private Turret() {
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);
        Logger.processInputs("Turret", turretInputs);
        updateMechanism();
    }

    void alignTurretToHub() {
        Rotation2d targetAngle = calculateTargetAngle();
        Rotation2d targetAngleAfterLimitCheck = limitAngle(targetAngle);
        turretIO.setTargetAngle(targetAngleAfterLimitCheck);
    }

    private Rotation2d calculateTargetAngle() {
        Pose2d currentPosition = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose();
        Translation2d difference = TurretConstants.HUB_POSITION.minus(currentPosition.getTranslation());
        double theta = Math.atan2(difference.getY(), difference.getX());
        double targetAngle = theta - currentPosition.getRotation().getRadians();
        return Rotation2d.fromRadians(targetAngle);
    }

    private Rotation2d limitAngle(Rotation2d targetAngle) {
        if (targetAngle.getDegrees() > TurretConstants.DEGREES_LIMIT)
            return Rotation2d.fromDegrees(targetAngle.getDegrees() - 360);
        if (targetAngle.getDegrees() < -TurretConstants.DEGREES_LIMIT)
            return Rotation2d.fromDegrees(targetAngle.getDegrees() + 360);
        return targetAngle;
    }

    private void updateMechanism() {
        TurretConstants.TURRET_LIGAMENT.setAngle(turretInputs.motorPositionDegrees);
        TurretConstants.TARGET_ANGLE_LIGAMENT.setAngle(turretInputs.profiledTargetPositionDegrees);
        Logger.recordOutput("Mechanisms/TurretMechanism", TurretConstants.TURRET_MECHANISM);
        Logger.recordOutput("Poses/Components/TurretPose", getTurretPose());
    }

    public Pose3d getTurretPose() {
        return new Pose3d(new Translation3d(), new Rotation3d(0, 0, Units.degreesToRadians(turretInputs.motorPositionDegrees)));
    }
}
