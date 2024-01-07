package frc.trigon.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;

public class TurretConstants {
    static final double DEGREES_LIMIT = 200;

    private static final double
            HUB_X = 8.248,
            HUB_Y = 4.176;
    static final Translation2d HUB_POSITION = new Translation2d(HUB_X, HUB_Y);

    private static final double
            TURRET_MECHANISM_WIDTH = 4,
            TURRET_MECHANISM_HEIGHT = 4;
    private static final double
            TURRET_ROOT_X = 2,
            TURRET_ROOT_Y = 2;
    private static final double
            TURRET_ROOT_LENGTH = 1,
            TURRET_ROOT_ANGLE = 0;
    private static final double MECHANISM_LINE_WIDTH = 10;
    static final Mechanism2d TURRET_MECHANISM = new Mechanism2d(TURRET_MECHANISM_WIDTH, TURRET_MECHANISM_HEIGHT);
    static final MechanismRoot2d
            TURRET_ROOT = TURRET_MECHANISM.getRoot("ZTurretRoot", TURRET_ROOT_X, TURRET_ROOT_Y),
            TARGET_ANGLE_ROOT = TURRET_MECHANISM.getRoot("TargetAngleRoot", TURRET_ROOT_X, TURRET_ROOT_Y);
    static final MechanismLigament2d
            TURRET_LIGAMENT = TURRET_ROOT.append(new MechanismLigament2d("TurretLigament", TURRET_ROOT_LENGTH, TURRET_ROOT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kBlue))),
            TARGET_ANGLE_LIGAMENT = TARGET_ANGLE_ROOT.append(new MechanismLigament2d("TargetAngleLigament", TURRET_ROOT_LENGTH, TURRET_ROOT_ANGLE, MECHANISM_LINE_WIDTH, new Color8Bit(Color.kGray)));

}
