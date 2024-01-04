package frc.trigon.robot.motorsimulation;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.utilities.Conversions;

public abstract class MotorSimulation {
    private PIDController pidController;
    private ProfiledPIDController profiledPIDController;
    private double voltageCompensationSaturation;
    private MotorSimulationConfiguration config;
    private double voltage = 0;

    public void applyConfiguration(MotorSimulationConfiguration config) {
        profiledPIDController = new ProfiledPIDController(config.pidConfigs.kP, config.pidConfigs.kI, config.pidConfigs.kD, new TrapezoidProfile.Constraints(config.motionMagicConfigs.maxVelocity, config.motionMagicConfigs.maxAcceleration));
        pidController = new PIDController(config.pidConfigs.kP, config.pidConfigs.kI, config.pidConfigs.kD);
        voltageCompensationSaturation = config.voltageCompensationSaturation;
        this.config = config;
        pidContinuousInput(config.pidConfigs.enableContinuousInput);
    }

    public void stop() {
        setVoltage(0);
    }

    public void setControl(VoltageOut voltageRequest) {
        setVoltage(voltageRequest.Output);
    }

    public void setControl(PositionVoltage positionVoltage) {
        double voltage = pidController.calculate(getPositionRevolutions() ,positionVoltage.Position * config.conversionFactor);
        setVoltage(voltage);
    }

    public void SetControl(MotionMagicVoltage motionMagicRequest) {
        double pidOutput = profiledPIDController.calculate(getPositionRevolutions(), motionMagicRequest.Position * config.conversionFactor);
        double feedforwardOutput = calculateFeedforward(config.feedForwardConfigs.kS, config.feedForwardConfigs.kG, config.feedForwardConfigs.kV, config.feedForwardConfigs.kA, profiledPIDController.getGoal().velocity, motionMagicRequest.Position * config.conversionFactor);
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

    abstract double calculateFeedforward(double ks, double kg, double kv, double ka, double targetVelocity, double targetPosition);

    abstract double getPositionRevolutions();

    abstract double getVelocityRevolutionsPerSecond();

    abstract double getCurrent();

    abstract void setInputVoltage(double voltage);

    private void setVoltage(double voltage) {
        double compensatedVoltage = MathUtil.clamp(voltage, -voltageCompensationSaturation, voltageCompensationSaturation);
        this.voltage = compensatedVoltage;
        setInputVoltage(compensatedVoltage);
    }

    private void pidContinuousInput(boolean enableContinuousInput) {
        if (enableContinuousInput) {
            pidController.enableContinuousInput(-(0.5 * config.conversionFactor), 0.5 * config.conversionFactor);
            profiledPIDController.enableContinuousInput(-(0.5 * config.conversionFactor), 0.5 * config.conversionFactor);
        } else {
            pidController.disableContinuousInput();
            profiledPIDController.disableContinuousInput();
        }
    }
}
