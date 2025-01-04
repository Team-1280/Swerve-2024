package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveMovementCommand extends Command {
    private final SwerveDriveSubsystem m_swerve; 
    private PhoenixPIDController m_thetaController;

    private final DoubleSupplier x_supplier;
    private final DoubleSupplier y_supplier;
    private final DoubleSupplier rot_supplier;

    private double x_val, y_val, rot_val;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle();
    private SwerveRequest request;

    private double speedMultiplier = 0.5;
    private double angularSpeedMultiplier = 0.8;
    private double driveDeadband = 0.1;

    class InputVisualization {
        public Mechanism2d mechanism;
        private MechanismRoot2d root;
        private MechanismLigament2d desiredVector;
        private MechanismLigament2d actualVector;

        InputVisualization() {
            this.mechanism = new Mechanism2d(2, 2, new Color8Bit(0, 0, 0));
            this.root = this.mechanism.getRoot("joystick", 1, 1);
            this.desiredVector = this.root.append(new MechanismLigament2d("desired", 1, 90));
            this.desiredVector.setColor(new Color8Bit(Color.kAliceBlue));
            this.actualVector = this.root.append(new MechanismLigament2d("actual", 1, 90));
            this.actualVector.setColor(new Color8Bit(Color.kRed));
        }

        void update(double desiredX, double desiredY, double actualX, double actualY) {
            this.desiredVector.setLength(Math.sqrt(desiredX * desiredX + desiredY * desiredY));
            this.desiredVector.setAngle(new Rotation2d(desiredY, desiredX));
            this.actualVector.setLength(Math.sqrt(actualX * actualX + actualY * actualY));
            this.actualVector.setAngle(new Rotation2d(actualY, actualX));
        }
    }

    private InputVisualization visualization;

    public SwerveMovementCommand(SwerveDriveSubsystem swerve,
        DoubleSupplier x_supplier,
        DoubleSupplier y_supplier,
        DoubleSupplier rot_supplier
    ) {
        this.addRequirements(swerve);

        this.m_swerve = swerve;
        this.x_supplier = x_supplier;
        this.y_supplier = y_supplier;
        this.rot_supplier = rot_supplier;
        
        this.visualization = new InputVisualization();
    }

    @Override
    public void initialize() {
        m_thetaController = new PhoenixPIDController(3.2, 0, 0);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        driveAngle.HeadingController = m_thetaController;

        SmartDashboard.putData(this.getName()+"/inputs", this.visualization.mechanism);
    }

    public void handleJoystickInput() {
        this.rot_val = MathUtil.applyDeadband(rot_supplier.getAsDouble() * this.angularSpeedMultiplier, 0.1);
        double x = x_supplier.getAsDouble();
        double y = y_supplier.getAsDouble(); 

        // deadband drive vector 
        double magnitude = MathUtil.applyDeadband(Math.sqrt(x*x + y*y), driveDeadband);
        if (magnitude == 0.0) {
            this.x_val = 0.0;
            this.y_val = 0.0;
        } else {
            this.x_val = x / Math.abs(x) * MathUtil.interpolate(0, this.speedMultiplier, Math.abs(x));
            this.y_val = y / Math.abs(y) * MathUtil.interpolate(0, this.speedMultiplier, Math.abs(y));
        }

        this.visualization.update(-x, y, -this.x_val, this.y_val);
    }

    @Override
    public void execute() {
        handleJoystickInput();

        this.request = drive
            .withVelocityX(this.x_val * Drivetrain.MAX_DRIVE_VOLTAGE)
            .withVelocityY(this.y_val * Drivetrain.MAX_DRIVE_VOLTAGE)
            .withRotationalRate(this.rot_val * Drivetrain.MAX_TURN_VOLTAGE);

        this.m_swerve.setControl(request);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType(this.getName());
        builder.addDoubleProperty("speed multiplier", () -> this.speedMultiplier, (k) -> this.speedMultiplier = k);
        builder.addDoubleProperty("rotational speed multiplier", () -> this.angularSpeedMultiplier, (k) -> this.angularSpeedMultiplier = k);
        builder.addDoubleProperty("rotational velocity (rad|s)", () -> this.rot_val * Drivetrain.MAX_TURN_VOLTAGE, null);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
