package frc.trigon.robot.motorsimulation;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.trigon.robot.utilities.Conversions;

public abstract class MotorSimulation {
    private PIDController pidController;
    private ProfiledPIDController profiledPIDController;
    private SimpleMotorFeedforward feedforward;
    private double voltageCompensationSaturation;
    private double voltage = 0;

    public void applyConfiguration(MotorSimulationConfiguration config) {
        profiledPIDController = new ProfiledPIDController(config.kP, config.kI, config.kD, new TrapezoidProfile.Constraints(config.maxVelocity, config.maxAcceleration));
        pidController = new PIDController(config.kP, config.kI, config.kD);
        feedforward = new SimpleMotorFeedforward(config.kS, config.kV, config.kA);
        voltageCompensationSaturation = config.voltageCompensationSaturation;
    }

    public void stop() {
        voltage = 0;
        setInputVoltage(0);
    }

    public void setVoltage(double voltage) {
        double compensatedVoltage = MathUtil.clamp(voltage, -voltageCompensationSaturation, voltageCompensationSaturation);
        voltage = compensatedVoltage;
        setInputVoltage(compensatedVoltage);
    }

    public void setControl(VoltageOut VoltageRequest) {
        setVoltage(VoltageRequest.Output);
    }

    public void setControl(PositionVoltage positionVoltage) {
        double voltage = pidController.calculate(positionVoltage.Position);
        setVoltage(voltage);
    }

    public void SetControl(MotionMagicVoltage MotionMagicRequest) {
        double pidOutput = profiledPIDController.calculate(getPositionRevolutions(), MotionMagicRequest.Position);
        double feedforwardOutput = feedforward.calculate(getVelocityRevolutionsPerSecond());
        double output = pidOutput + feedforwardOutput;
        setVoltage(output);
    }

    public void setControl(DutyCycleOut powerRequest) {
        double voltage = Conversions.compensatedPowerToVoltage(powerRequest.Output, voltageCompensationSaturation);
        setVoltage(voltage);
    }


    public double getVoltage() {
        return voltage;
    }

    abstract double getPositionRevolutions();

    abstract double getVelocityRevolutionsPerSecond();

    abstract double getCurrent();

    abstract void setInputVoltage(double voltage);
}
