// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class RobotMath {
    public static class Elevator {
        // Converting Linear Velocity to Angular Velocity (for SIM)
        

        // Rotations to Distance
        public static Distance convertRotationsToDistance(Angle rotations) {
            // You need to convert double into "Distance", so you must use the "Meters" class
            return Meters.of(rotations.in(Rotations) * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius) / ElevatorConstants.kElevatorGearing);
        }

        // Distance to Rotations
        public static Angle convertDistanceToRotations(Distance distance) {
            return Rotations.of(distance.in(Meters) / (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius) * ElevatorConstants.kElevatorGearing);
        }
    }

    public static class Arm {
        public static Angle convertArmAngleToSensorUnits(Angle measurement) {
            return Rotations.of(measurement.in(Rotations) * ArmConstants.kArmReduction);
        }

        public static Angle convertSensorUnitsToArmAngle(Angle measurement) {
            return Rotations.of(measurement.in(Rotations) / ArmConstants.kArmReduction);
        }
    }
}
