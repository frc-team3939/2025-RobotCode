package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


//Make sure that every single one of these variables are correct before troubleshooting the robot further! 
    

public final class Constants {

    public static final class ModuleConstants {

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.56;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    //You're funny, funny lookin
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(18.5);
        // Distance between the center of the right and left wheels in inches.
        public static final double kWheelBase = Units.inchesToMeters(13.125);
        // Distance between the center of the front and back wheels in inches.
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //FL
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //FR
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //BL
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //BR

        //Plug into any SparkMax with a USB-C cable and use the utility on the desktop to check the port IDs of each motor. 
        public static final int kFrontLeftDriveMotorPort = 40;   //FL
        public static final int kBackLeftDriveMotorPort = 24;    //BL
        public static final int kFrontRightDriveMotorPort = 52;  //FR
        public static final int kBackRightDriveMotorPort = 42;   //BR

        public static final int kFrontLeftTurningMotorPort = 41;     //FL
        public static final int kBackLeftTurningMotorPort = 43;      //BL
        public static final int kFrontRightTurningMotorPort = 57;    //FR
        public static final int kBackRightTurningMotorPort = 27;     //BR

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;
        
        //Check Phoenix Tuner X for these values when connected to robot
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 3;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 5;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 4;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

        //Check Andymark/Ryan's math for max speed in METERS PER SECOND
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.121;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;


        //Limits tele-op max speed, acceleration, etc.
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 1;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kTopHalfButtonBoardPort = 1;
        public static final int kBottomHalfButtonBoardPort = 2;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        // public static final int kLeftTriggerAxis = 3;
        // public static final int kRightTriggerAxis = 4;
        public static void main(String[] args) {
            
        }
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.08;
    }

}
