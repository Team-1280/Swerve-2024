package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends Command {
    private final VisionSubsystem m_visionSubsystem;

    /**
     * Creates a new VisionAlignCommand.
     *
     * @param visionSubsystem self explanatory.
     */
    public VisionAlignCommand(VisionSubsystem visionSubsystem) {
        this.m_visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void execute() {
        m_visionSubsystem.moveToAprilTagPeriodic();
    }

    @Override
    public boolean isFinished() {
        return m_visionSubsystem.isAlignedWithAprilTag();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_visionSubsystem.stop();
        }
    }
}
