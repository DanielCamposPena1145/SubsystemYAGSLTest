// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotMath.Arm;

public class ArmSubsystem extends SubsystemBase {
  
  private final SparkFlex m_motor = new SparkFlex(ArmConstants.MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final ArmFeedforward m_armFeedforward = new ArmFeedforward(
    ArmConstants.kArmkS,
    ArmConstants.kArmkG,
    ArmConstants.kArmkV,
    ArmConstants.kArmkA);

  private final ProfiledPIDController m_controller = new ProfiledPIDController(
    ArmConstants.kArmkP,
    ArmConstants.kArmkI,
    ArmConstants.kArmKD,
    new Constraints(ArmConstants.kArmMaxVelocity, ArmConstants.kArmMaxAcceleration));
  
  
  public ArmSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(ArmConstants.ArmStallCurrentLimit)
      .openLoopRampRate(ArmConstants.ArmRampRate)
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .closedLoop.maxMotion
        .maxVelocity(ArmConstants.kArmMaxVelocity)
        .maxAcceleration(ArmConstants.kArmMaxAcceleration)
        .allowedClosedLoopError(ArmConstants.kArmAllowedClosedLoopError);
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Angle getAngle() {
    return Arm.convertSensorUnitsToArmAngle(Rotations.of(m_encoder.getPosition()));
  }

  public AngularVelocity getVelocity() {
    return Arm.convertSensorUnitsToArmAngle(Rotations.of(m_encoder.getVelocity())).per(Minute);
  }

  // REACH GOAL
  /* This is the core method for moving the arm to a target position. It gets called repeatedly while the command is running. */
  public void reachGoal(double goalDegrees) {
    double goal = Arm.convertArmAngleToSensorUnits(Degrees.of(goalDegrees)).in(Rotations);

    m_motor.setVoltage(m_armFeedforward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity) + m_controller.calculate(m_encoder.getPosition(), goal));
    /* goalDegrees - The desired angle in degrees
     *      a. It's converted to encoder rotations
     * 
     * m_controller.calculate - Generates a correction voltage using PID to get from current position to goal.
     * m_armFeedforward.calculate - Adds in feedforward, which predicts the effort needed to hold or move the arm at a certain velocity or position, even if there were no error.
     */
  }

  // COMMAND
  public Command setGoal(double goalDegrees) {
    return run(()-> reachGoal(goalDegrees));
    /* This returns a Command that will continuously call "reachGoal()" until the command ends
     * eg.
     *  arm.setGoal(45)
     *      --> Tells he arm to move toward 45 degrees but this is a CONTINUOUS command.
     */
  }

  public Command setArmAngle(double goalDegrees) {
    // This command wraps "setGoal()" and gives it a stopping condition
    return setGoal(goalDegrees).until(()-> aroundAngle(goalDegrees));
    /* It runs the arm to the target angle UNTIL "aroundAngle(...)" returns true, which means
     * the arm si close enough to the target
     * 
     * It's a self-ending command.
     */
  }

  // Default Tolerance method
  public boolean aroundAngle(double degrees) {
    return aroundAngle(degrees, ArmConstants.kArmDefaultTolerance);
  }

  // Custom Tolerance
  public boolean aroundAngle(double degrees, double tolerance) {
    return MathUtil.isNear(degrees, getAngle().in(Degrees), tolerance);
  }

  public void stop() {
    m_motor.set(0.0);
  }


  @Override
  public void periodic() {
    // SmartDashboard.putData("Arm Angle", getAngle());
  }
  
}
