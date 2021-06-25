// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    public static final class DriveControllers {
        public static final int frontLeft = 5;
        public static final int frontRight = 6;
        public static final int backLeft = 4;
        public static final int backRight = 7;
    }

    public static final class DriveEncoders {
        public static final int frontLeftA = 0;
        public static final int frontLeftB = 1;
        public static final int frontRightA = 8;
        public static final int frontRightB = 9;
        public static final int backLeftA = 4;
        public static final int backLeftB = 5;
        public static final int backRightA = 6;
        public static final int backRightB = 7;
    }

    public static final class TurnControllers {
        public static final int frontLeft = 19;
        public static final int frontRight = 13;
        public static final int backLeft = 4;
        public static final int backRight = 8;
    }

    public static final class TurnEncoders {
        public static final int frontLeftA= 2;
        public static final int frontLeftB = 3;
        public static final int frontRightA = 4;
        public static final int frontRightB = 5;
        public static final int backLeftA = 6;
        public static final int backLeftB = 7;
        public static final int backRightA = 8;
        public static final int backRightB = 9;
    }

    public static final class ShooterControllers {
        public static final int leftShooter = 2;
        public static final int rightShooter = 0;
    }

    public static final class ShooterConstants {
        public static final double shooterSpeed = 1;
    }

    public static final class ElevatorControllers {
        public static final int elevator = 1;
    }

    public static final class ElevatorConstants {
        public static final double elevatorSpeed = 0.4;
    }

    public static final class EncoderRes {
        public static final int driveRes = 80; //AndyMark CIMcoder
        public static final int turnRes = 4096; //VEX Integrated Motor Encoder
    }

    public static final class Chassis {
        public static final double front2Back = 0.6858; //in meters
        public static final double left2Right = 0.5842; //in meters

        public static final double wheelRadius = 0.0381; //in meters

        public static final double posX = front2Back / 2; //DO NOT EDIT
        public static final double posY = left2Right / 2; //DO NOT EDIT

        public static final double frontLeftX = 0.635;
        public static final double frontLeftY = 0.0762;
        public static final double frontRightX = 0.635;
        public static final double frontRightY = -0.5207;
        public static final double backLeftX = -0.0635;
        public static final double backLeftY = 0.0762;
        public static final double backRightX = -0.0635;
        public static final double backRightY = -0.5207;
    }
}
