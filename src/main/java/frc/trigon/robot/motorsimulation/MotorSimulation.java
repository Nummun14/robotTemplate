package frc.trigon.robot.motorsimulation;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.utilities.Conversions;

import java.util.ArrayList;
import java.util.List;

/**
 * A class that simulates a motor and its PID and feedforward.
 */
public abstract class MotorSimulation {
    private static final List<MotorSimulation> REGISTERED_SIMULATIONS = new ArrayList<>();

    static {
        new Notifier(MotorSimulation::updateRegisteredSimulations).startPeriodic(RobotConstants.PERIODIC_TIME_SECONDS);
    }

    private PositionVoltage positionVoltageRequest;
    private MotionMagicVoltage motionMagicRequest;
    private PIDController pidController;
    private ProfiledPIDController profiledPIDController;
    private double voltageCompensationSaturation;
    private MotorSimulationConfiguration config;
    private double voltage = 0;

    public MotorSimulation() {
        REGISTERED_SIMULATIONS.add(this);
    }

    public void applyConfiguration(MotorSimulationConfiguration config) {
        profiledPIDController = new ProfiledPIDController(config.pidConfigs.kP, config.pidConfigs.kI, config.pidConfigs.kD, new TrapezoidProfile.Constraints(config.motionMagicConfigs.maxVelocity, config.motionMagicConfigs.maxAcceleration));
        pidController = new PIDController(config.pidConfigs.kP, config.pidConfigs.kI, config.pidConfigs.kD);
        voltageCompensationSaturation = config.voltageCompensationSaturation;
        this.config = config;
        enablePIDContinuousInput(config.pidConfigs.enableContinuousInput);
    }

    public void stop() {
        setVoltage(0);
    }

    public void setControl(VoltageOut voltageRequest) {
        setVoltage(voltageRequest.Output);
    }

    public void setControl(PositionVoltage positionVoltageRequest) {
        this.positionVoltageRequest = positionVoltageRequest;
        double voltage = pidController.calculate(getPositionRevolutions() * config.conversionFactor, positionVoltageRequest.Position);
        setVoltage(voltage);
    }

    public void setControl(MotionMagicVoltage motionMagicRequest) {
        this.motionMagicRequest = motionMagicRequest;
        double pidOutput = profiledPIDController.calculate(getPositionRevolutions() * config.conversionFactor, motionMagicRequest.Position);
        double feedforwardOutput = calculateFeedforward(config.feedForwardConfigs, Units.rotationsToRadians(motionMagicRequest.Position / config.conversionFactor), profiledPIDController.getGoal().velocity);
        double output = pidOutput + feedforwardOutput;
        setVoltage(output);
    }

    public void setControl(DutyCycleOut dutyCycleRequest) {
        double voltage = Conversions.compensatedPowerToVoltage(dutyCycleRequest.Output, voltageCompensationSaturation);
        setVoltage(voltage);
    }

    public double getVoltage() {
        return voltage;
    }

    public double getPosition() {
        return getPositionRevolutions() * config.conversionFactor;
    }

    public double getVelocity() {
        return getVelocityRevolutionsPerSecond() * config.conversionFactor;
    }

    private void setVoltage(double voltage) {
        double compensatedVoltage = MathUtil.clamp(voltage, -voltageCompensationSaturation, voltageCompensationSaturation);
        this.voltage = compensatedVoltage;
        setInputVoltage(compensatedVoltage);
    }

    private void enablePIDContinuousInput(boolean enableContinuousInput) {
        if (enableContinuousInput) {
            pidController.enableContinuousInput(-(0.5 * config.conversionFactor), 0.5 * config.conversionFactor);
            profiledPIDController.enableContinuousInput(-(0.5 * config.conversionFactor), 0.5 * config.conversionFactor);
        } else {
            pidController.disableContinuousInput();
            profiledPIDController.disableContinuousInput();
        }
    }

    private static void updateRegisteredSimulations() {
        for (MotorSimulation motorSimulation : REGISTERED_SIMULATIONS) {
            motorSimulation.setControl(motorSimulation.positionVoltageRequest);
            motorSimulation.setControl(motorSimulation.motionMagicRequest);
            motorSimulation.updateMotor();
        }
    }

    /**
     * Calculates the feedforward
     *
     * @param feedForwardConfiguration The feedforward configuration
     * @param targetPositionRadians    The target position in radians
     * @param targetVelocity           The target velocity
     * @return The calculated feedforward voltage
     */
    abstract double calculateFeedforward(MotorSimulationConfiguration.FeedForwardConfigs feedForwardConfiguration, double targetPositionRadians, double targetVelocity);

    abstract double getPositionRevolutions();

    abstract double getVelocityRevolutionsPerSecond();

    abstract double getCurrent();

    abstract void setInputVoltage(double voltage);

    abstract void updateMotor();
}
