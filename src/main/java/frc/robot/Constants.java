// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import frc.robot.RobotMath.Arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/** Add your docs here. */
public final class Constants {
    public final class OIConstants {
        public final static int kOperatorControllerPort = 0;
    }

    public final class ElevatorConstants {
        public final static int MOTOR_ID = 13;
        //ProfiledPID
        public final static double kElevatorkP = 0;
        public final static double kElevatorkI = 0;
        public final static double kElevatorkD = 0;
        public final static double kMaxVelocity = Meters.of(2).per(Second).in(MetersPerSecond);
        public final static double kMaxAcceleration = Meters.of(1).per(Second).per(Second).in(MetersPerSecondPerSecond);
        //Feedforward
        public final static double kElevatorkS = 0;
        public final static double kElevatorkG = 0;
        public final static double kElevatorkV = 0;
        public final static double kElevatorkA = 0;
        // Loop Ramp Rate
        public final static double kElevatorRampRate = 0.1;
        // Elevator Physical Data
        public final static double kElevatorDrumRadius = Inches.of(2).in(Meters);
        public final static double kElevatorGearing = 12;
        // Command Height
        public static final double kElevatorDefaultTolerance = Inches.of(1).in(Meters); // In case we want multiply tolerances, we create a default tolerance
    }

    public final class ArmConstants {
        public final static int MOTOR_ID = 12;
        public final static int kArmReduction = 12;
        public final static int ArmStallCurrentLimit = 45;
        // Feedforward Values
        public final static double kArmkS = 0;
        public final static double kArmkG = 0;
        public final static double kArmkV = 0;
        public final static double kArmkA = 0;
        // ProfiledPID Values
        public final static double kArmkP = 0;
        public final static double kArmkI = 0;
        public final static double kArmKD = 0;
            // Constraints
            // The PID Controller and motor only understand "rotations" (what the encoder reads), not degrees. So this ensures your constraints are in the same unit the motor controller expects.
            public final static double kArmMaxVelocity = Arm.convertArmAngleToSensorUnits(Degrees.of(50)).per(Second).in(RPM);  // The arm is allowed to move up to 50 degrees per second -> In RPM
            public final static double kArmMaxAcceleration = Arm.convertArmAngleToSensorUnits(Degrees.of(180)).per(Second).per(Second).in(RPM.per(Second)); // The arm is allowed to accelerate up to 180 degrees per second^2 -> In RPM

        public final static double ArmRampRate = 0.2;
        public final static double kArmAllowedClosedLoopError = 0.8;
        public final static double kArmDefaultTolerance = 1;

    }
    

    

}
