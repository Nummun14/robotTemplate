package frc.trigon.robot.motorsimulation;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public abstract class MotorSimulation {
    private PIDController pidController;
    private ProfiledPIDController profiledPIDController;
    private SimpleMotorFeedforward feedforward;
    private double voltage = 0;

    public void applyConfiguration(MotorSimulationConfiguration config) {
        this.profiledPIDController = new ProfiledPIDController(config.kP, config.kI, config.kD, config.constraints);
        this.pidController = new PIDController(config.kP, config.kI, config.kD);
        this.feedforward = new SimpleMotorFeedforward(config.kS, config.kV, config.kA);
    }

    public void stop() {
        setInputVoltage(0);
    }

    public void voltageRequest(VoltageOut VoltageRequest) {
        this.voltage = VoltageRequest.Output;
        setInputVoltage(VoltageRequest.withOutput(voltage).Output);
    }

    public void positionVoltageRequest(PositionVoltage positionVoltage) {
        double voltage = pidController.calculate(positionVoltage.Position);
        this.voltage = voltage;
        setInputVoltage(voltage);
    }

    public void motionMagicRequest(MotionMagicVoltage MotionMagicRequest) {
        double pidOutput = profiledPIDController.calculate(getPositionRevolutions(), MotionMagicRequest.Position);
        double feedforwardOutput = feedforward.calculate(getVelocityRevolutionsPerSecond());
        double output = pidOutput + feedforwardOutput;
        this.voltage = output;
        voltageRequest(new VoltageOut(output));
    }

    public double getVoltage() {
        return voltage;
    }

    abstract double getPositionRevolutions();

    abstract double getVelocityRevolutionsPerSecond();

    abstract double getCurrent();

    abstract void setInputVoltage(double voltage);
}
