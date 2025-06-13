// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkFlex m_motor = new SparkFlex(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  
  private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.kElevatorkP, 
                                                                                ElevatorConstants.kElevatorkI, 
                                                                                ElevatorConstants.kElevatorkD, 
                                                                                new Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));

  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(ElevatorConstants.kElevatorkS, 
                                                                            ElevatorConstants.kElevatorkG, 
                                                                            ElevatorConstants.kElevatorkV,
                                                                            ElevatorConstants.kElevatorkA);




  public ElevatorSubsystem() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit(45).openLoopRampRate(ElevatorConstants.kElevatorRampRate);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double getPositionMeters() {
    return m_encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius) / ElevatorConstants.kElevatorGearing;
  }

  public double getVelocityMetersPerSecond() {
    // getVelocity() is in RPM, convert it to Meters per second
    /* From Minutes to Seconds, divide RPM Velocity by 60
     * From Rotations to Meters, 1 rotation is 2*pi*r (Circumference of Drum)
     * Multiply Gear ratio
     */
    return (m_encoder.getVelocity() / 60) * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius) / ElevatorConstants.kElevatorGearing;
  }

  // To Update the elevator height, we need a voltage, the voltage from PID and FF combined, create the needed voltage to update the elevator height
  public void reachGoal(double goal) {
    // Using "calculate()" uses a set velocity, assuming acc is 0 which we do not want, so instead we use "calculateWithVelocities()".
    // The "nextVelocity" is the velocity you will retrieve from the PID controller
    double voltsOutput = MathUtil.clamp((m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(), m_controller.getSetpoint().velocity)
    // Now we need the voltage from the PID
    + m_controller.calculate(getPositionMeters(), goal)), -7, 7);
    // We need to be aware that the voltage oupput should always be a safe voltage output, never exceeding a certain volt, so we use "MathUtil.clamp" which takes double value, low, high. Note: Value is our FF + PID Voltage output

    // Now we need to set the voltage to the motor
    m_motor.setVoltage(voltsOutput);
  }

  // Since we using command based programming, we need to create a command called:
  public Command setGoal(double goal) {
    // We need to return something, and we use the "run" command to supply voltage constantly
    return run(()-> reachGoal(goal));
    // IMPORTANT: This command will keep running, it will NOT STOP so you must create something that stops it
  }

  // Condition to interrupt setGoal Command
  public Command setElevatorHeight(double height) {
    // This returns setGoal, however, with a condition to interrupt the command or end it
    return setGoal(height).until(()-> aroundHeight(height));
  }

  // Boolean to check around height of elevator - desired vs actual
  public boolean aroundHeight(double height) {
    return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
  }


  public boolean aroundHeight(double height, double tolerance) {
    return MathUtil.isNear(height, getPositionMeters(), tolerance);
  }


  @Override
  public void periodic() {
  }
}
