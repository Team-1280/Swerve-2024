package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;

    /**
     * Creates a new {@link VisionSubsystem}.
     */
    public VisionSubsystem(SwerveDriveSubsystem swerve) {
        this.m_swerveDriveSubsystem = swerve;
    }

    /**
     * Moves to the primary April Tag periodically...
     * Mostly temporary, might need to fix?
     * TODO: 1/4/2025
     */
    public void moveToAprilTagPeriodic() {
        Pose3d aprilTagPosition = LimelightHelpers.getTargetPose3d_CameraSpace("");

        double tagRotation = aprilTagPosition.getRotation().getY();
        double robotRotation = this.m_swerveDriveSubsystem.getPose().getRotation().getRadians();
        // Find the shortest path to said rotation.....
        double desiredRotation = Math.atan2(Math.sin(tagRotation - robotRotation), Math.cos(tagRotation - robotRotation));
        double angularSpeed = Math.max(-Constants.Vision.kMaxAngularSpeed, Math.min(Constants.Vision.kMaxAngularSpeed, Constants.Vision.kProportionalRotation * desiredRotation));

        if (Math.abs(desiredRotation) < Constants.Vision.tolerance) {
            return;
        }

        this.m_swerveDriveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, angularSpeed));
    }

    @Override
    public void periodic() {
        // Temporary
        moveToAprilTagPeriodic();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}