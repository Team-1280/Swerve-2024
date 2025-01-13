package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {

  public SwerveDriveSubsystem() {
    super(
      Drivetrain.DrivetrainConstants, 
      Drivetrain.moduleConstants
    );
    configureAutoBuilder();
  }

  public void configureAutoBuilder() {
        try {
          AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry
            this::getRobotRelativeSpeeds, //ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID
              new PIDConstants(5.0, 0.0, 0.0), // Angle PID
              5, // Max speed in m/s
              0, // <--CHANGE TO YOUR DRIVE BASE RADIUS (distance from center of robot to furthest module in meters)
              new ReplanningConfig()
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this
          );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
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
