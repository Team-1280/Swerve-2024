package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {

  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  public SwerveDriveSubsystem() {
    super(
      Drivetrain.DrivetrainConstants, 
      Drivetrain.moduleConstants
    );
  }

  private SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();

  public Pose2d getPose(){
    return this.getState().Pose;
  }

  public void resetPose(Pose2d newPose){
    this.seedFieldRelative(newPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates);       
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    this.setControl(drive.withSpeeds(speeds));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return this.run(() -> this.setControl(requestSupplier.get()));
  }

  public void periodic() {
    for(int i=0; i<4; i++) {
      SmartDashboard.putData("encoders ("+i+")", this.getModule(i).getCANcoder());
    }
  }
}
