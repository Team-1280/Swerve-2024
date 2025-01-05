package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private int up = 2;

    /**
     * Creates a new {@link VisionSubsystem}.
     */
    public VisionSubsystem(SwerveDriveSubsystem swerve) {
        this.m_swerveDriveSubsystem = swerve;
        this.up = 2;
    }

    /**
     * Moves to the primary April Tag periodically...
     * Mostly temporary, might need to fix?
     * TODO: 1/4/2025
     */
    public void moveToAprilTagPeriodic() {
        Pose3d killMe = LimelightHelpers.getTargetPose3d_CameraSpace("");

        double tagRotation = killMe.getRotation().getY();
        double robotRotation = this.m_swerveDriveSubsystem.getPose().getRotation().getRadians();
        // Find the shortest path to said rotation.....
        double desiredRotation = Math.atan2(Math.sin(tagRotation - robotRotation), Math.cos(tagRotation - robotRotation));
        double angularSpeed = Math.max(-Constants.Vision.kMaxAngularSpeed, Math.min(Constants.Vision.kMaxAngularSpeed, Constants.Vision.kProportionalRotation * desiredRotation));

        SmartDashboard.putNumber("robotRotation", robotRotation);
        SmartDashboard.putNumber("cewdgyuewyuew", tagRotation);
        SmartDashboard.putNumber("llehyugfu3yg", angularSpeed);

        // if (Math.abs(robotRotation - desiredRotation) < Constants.Vision.tolerance) {
        //     return;
        // }
        //this.m_swerveDriveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, angularSpeed));
        drive.withRotationalRate((angularSpeed * 0.8) * Drivetrain.MAX_TURN_VOLTAGE);
        this.m_swerveDriveSubsystem.setControl(drive);
    }

    @Override
    public void periodic() {
        // Temporary
        up++;
        if (up < 10) return;
        moveToAprilTagPeriodic();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}