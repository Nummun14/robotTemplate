package frc.trigon.robot.subsystems.turret.simulationturret;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.trigon.robot.motorsimulation.MotorSimulationConfiguration;
import frc.trigon.robot.motorsimulation.SimpleMotorSimulation;
import frc.trigon.robot.subsystems.turret.TurretIO;

public class SimulationTurretConstants extends TurretIO {
    private static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    private static final int MOTOR_AMOUNT = 1;
    private static final DCMotor MOTOR_GEARBOX = DCMotor.getFalcon500Foc(MOTOR_AMOUNT);
    private static final double MOMENT_OF_INERTIA = 0.003;
    private static final double GEAR_RATIO = 100;
    private static final double
            MAX_MOTOR_VELOCITY = 1,
            MAX_MOTOR_ACCELERATION = 1;
    private static final double
            P = 1000,
            I = 1000,
            D = 0;
    private static final double
            KS = 0,
            KV = 0,
            KA = 0;
    static final SimpleMotorSimulation MOTOR = new SimpleMotorSimulation(
            MOTOR_GEARBOX,
            GEAR_RATIO,
            MOMENT_OF_INERTIA
    );

    static {
        MotorSimulationConfiguration config = new MotorSimulationConfiguration();
        config.pidConfigs.kP = P;
        config.pidConfigs.kI = I;
        config.pidConfigs.kD = D;

        config.feedForwardConfigs.kS = KS;
        config.feedForwardConfigs.kV = KV;
        config.feedForwardConfigs.kA = KA;

        config.motionMagicConfigs.maxVelocity = MAX_MOTOR_VELOCITY;
        config.motionMagicConfigs.maxAcceleration = MAX_MOTOR_ACCELERATION;

        config.voltageCompensationSaturation = VOLTAGE_COMPENSATION_SATURATION;

        MOTOR.applyConfiguration(config);
    }
}
