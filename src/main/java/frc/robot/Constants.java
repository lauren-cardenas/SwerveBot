// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    public static final class DriveControllers {
        public static final int frontLeft = 0;
        public static final int frontRight = 1;
        public static final int backLeft = 2;
        public static final int backRight = 3;
    }

    public static final class DriveEncoders {
        public static final int frontLeftA = 0;
        public static final int frontLeftB = 1;
        public static final int frontRightA = 2;
        public static final int frontRightB = 3;
        public static final int backLeftA = 4;
        public static final int backLeftB = 5;
        public static final int backRightA = 6;
        public static final int backRightB = 7;
    }

    public static final class TurnControllers {
        public static final int frontLeft = 0;
        public static final int frontRight = 1;
        public static final int backLeft = 2;
        public static final int backRight = 3;
    }

    public static final class EncoderRes {
        public static final int driveRes = 80; //AndyMark CANcoder
        public static final int turnRes = 4096; //VEX Integrated Motor Encoder
    }

    public static final class Chassis {
        public static final double front2Back = 1; //in meters
        public static final double left2Right = 1; //in meters

        public static final double wheelRadius = 0.0508; //in meters

        public static final double posX = front2Back / 2; //DO NOT EDIT
        public static final double posY = left2Right / 2; //DO NOT EDIT
    }
}
