// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

  private final XboxController m_opController = new XboxController(OIConstants.kOperatorControllerPort);


  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();


  public RobotContainer() {
    configureBindings();

    // Set default elevator command
    elevator.setDefaultCommand(elevator.setElevatorHeight(0));
    arm.setDefaultCommand(arm.setArmAngle(80));
    
  }

  private void configureBindings() {
    // Elevator Buttons
    new JoystickButton(m_opController, 1).whileTrue(elevator.setElevatorHeight(4));
    
    // Arm Buttons
    new JoystickButton(m_opController, 2).whileTrue(arm.setArmAngle(45));

    // Elevator & Arm Sequential Command -> L4
    new JoystickButton(m_opController, 3).whileTrue(elevator.setElevatorHeight(5).andThen(arm.setArmAngle(70)));

    // I'm experimenting with something ignore this I haven't tested it.
    // Elevator * Arm (Default 70 Degrees or 40 Degrees)
    // new JoystickButton(m_opController, 5).whileTrue(
    //   Commands.repeatingSequence(
    //     Commands.runOnce(()-> {
    //       if (!elevator.aroundHeight(6)) {
    //         elevator.setElevatorHeight(6).schedule();
    //       }
    //     }),
    //     Commands.waitSeconds(0.2),
    //     Commands.run(()-> {
    //       if (m_opController.getRawButton(7)) {
    //         arm.setArmAngle(40).schedule();
    //       } else {
    //         arm.setArmAngle(70).schedule();
    //       }
    //     })
    //   )
    // );


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
