package frc.robot;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.subsystem.climber.ClimberStates;
import frc.robot.subsystem.climber.ClimberSubsystem;
import frc.robot.subsystem.drivetrain.Drivetrain;
import frc.robot.subsystem.drivetrain.DrivetrainConstants;
import frc.robot.subsystem.drivetrain.control.PositionalControl;
import frc.robot.subsystem.drivetrain.control.VelocityFOC;
import frc.robot.subsystem.drivetrain.control.YawLockFOC;
import frc.robot.subsystem.feeder.FeederStates;
import frc.robot.subsystem.feeder.FeederSubsystem;
import frc.robot.subsystem.intake.IntakeStates;
import frc.robot.subsystem.intake.IntakeSubsystem;
import frc.robot.subsystem.mop.MopStates;
import frc.robot.subsystem.mop.MopSubsystem;
import frc.robot.subsystem.shooter.ShooterSubsystem;

import java.util.ArrayList;

public class Superstructure {
    private RobotStates robotState;

    private DriverStation.Alliance alliance;

    private Pose2d CLIMB_LEFT = new Pose2d();
    private Pose2d CLIMB_RIGHT = new Pose2d();
    private Pose2d FERRY_TARGET_LEFT = new Pose2d();
    private Pose2d FERRY_TARGET_RIGHT = new Pose2d();
    private Pose2d HOPPER_TARGET = new Pose2d();

    private Pose2d NEAR_FERRY_LEFT = new Pose2d();
    private Pose2d NEAR_FERRY_RIGHT = new Pose2d();
    private Pose2d NEAR_HOPPER = new Pose2d();

    private final ArrayList<Pose2d> NEAR_TARGET_ARRAY = new ArrayList<>();
    private final ArrayList<Pose2d> NEAR_CLIMBER_ARRAY = new ArrayList<>();

    private Pose2d shooterTarget = new Pose2d();

    private Rotation2d lockAngle;
    private boolean lockFirstTick;

    public void setRobotState(RobotStates robotState) {
        this.robotState = robotState;
    }

    private enum ShooterTargetLockMode{
        AutoAlign,
        None
    }

    private final SendableChooser<ShooterTargetLockMode> lockShooterChooser;
    private ShooterTargetLockMode targetLockOn;

    public Superstructure(){
        setRobotState(RobotStates.Default);
        ChoreoAllianceFlipUtil.setYear(2026);
        setAlliance(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));

        lockShooterChooser = new SendableChooser<>();
        lockShooterChooser.setDefaultOption("Auto Align On", ShooterTargetLockMode.AutoAlign);
        lockShooterChooser.addOption("Auto Align Off", ShooterTargetLockMode.None);
        lockShooterChooser.onChange(this::setTargetLock);

        lockFirstTick = false;
        lockAngle = new Rotation2d();
    }

    public void readPeriodic() {
        Drivetrain.getInstance().readPeriodic();
        ClimberSubsystem.getInstance().readPeriodic();
        FeederSubsystem.getInstance().readPeriodic();
        IntakeSubsystem.getInstance().readPeriodic();
        MopSubsystem.getInstance().readPeriodic();
        ShooterSubsystem.getInstance().readPeriodic();
    }

    public void writePeriodic() {
        Pose2d robotPose =  Drivetrain.getInstance().getPose();

        // Shooter updated
        shooterTarget = robotPose.nearest(NEAR_TARGET_ARRAY);
        if (shooterTarget == NEAR_FERRY_LEFT) {
            shooterTarget = FERRY_TARGET_LEFT;
        } else if (shooterTarget == NEAR_FERRY_RIGHT) {
            shooterTarget = FERRY_TARGET_RIGHT;
        } else if (shooterTarget == NEAR_HOPPER) {
            shooterTarget = HOPPER_TARGET;
        }
        ShooterSubsystem.getInstance().setTarget(robotPose, shooterTarget);

        Rotation2d targetLock = new Rotation2d(
            robotPose.getX() - shooterTarget.getX(),
            robotPose.getY() - shooterTarget.getY()
        );

        // Drivetrain Input
        double horizontalInput = IO.getJoystick(Controls.horizontalVelocity).getAsDouble();
        double verticalInput = IO.getJoystick(Controls.verticalVelocity).getAsDouble();
        double omegaInput = IO.getJoystick(Controls.omega).getAsDouble();

        if (alliance == DriverStation.Alliance.Blue) {
            horizontalInput *= -1;
            verticalInput *= -1;
        }

        horizontalInput = MathUtil.applyDeadband(horizontalInput,.05);
        verticalInput = MathUtil.applyDeadband(verticalInput,.05);
        omegaInput = MathUtil.applyDeadband(omegaInput,.05);

        horizontalInput = Math.pow(horizontalInput,3);
        verticalInput = Math.pow(verticalInput,3);
        omegaInput = Math.pow(omegaInput,3);

        LinearVelocity horizontalVelocity = LinearVelocity.ofBaseUnits(horizontalInput * DrivetrainConstants.MAX_VELOCITY, Units.MetersPerSecond);
        LinearVelocity verticalVelocity = LinearVelocity.ofBaseUnits(verticalInput * DrivetrainConstants.MAX_VELOCITY, Units.MetersPerSecond);
        AngularVelocity omegaVelocity = AngularVelocity.ofBaseUnits(omegaInput * DrivetrainConstants.MAX_ANGULAR_VELOCITY, Units.RotationsPerSecond);

        switch (robotState) {
            case Default -> {
                ClimberSubsystem.getInstance().setState(ClimberStates.retract);
                IntakeSubsystem.getInstance().setState(IntakeStates.StoredOff);

                manualDrive(robotPose, horizontalVelocity, verticalVelocity, omegaVelocity);
            }
            case IntakeShooting -> {
                ClimberSubsystem.getInstance().setState(ClimberStates.retract);
                IntakeSubsystem.getInstance().setState(IntakeStates.Deployed);

                if (targetLockOn == ShooterTargetLockMode.AutoAlign) {
                    Drivetrain.getInstance().setControl(
                        new YawLockFOC(
                            horizontalVelocity,
                            verticalVelocity,
                            targetLock
                        )
                    );
                }else{
                    manualDrive(robotPose, horizontalVelocity, verticalVelocity, omegaVelocity);
                }
                if (Drivetrain.getInstance().getControl().atSetpoint()) {
                    FeederSubsystem.getInstance().setState(FeederStates.FEED);
                    MopSubsystem.getInstance().setState(MopStates.FEED);
                }else{
                    MopSubsystem.getInstance().setState(MopStates.OFF);
                    FeederSubsystem.getInstance().setState(FeederStates.OFF);
                }
            }
            case Intake -> {
                ClimberSubsystem.getInstance().setState(ClimberStates.retract);
                IntakeSubsystem.getInstance().setState(IntakeStates.Deployed);
                manualDrive(robotPose, horizontalVelocity, verticalVelocity, omegaVelocity);
            }
            case AligningClimb -> {
                Pose2d alignPose = robotPose.nearest(NEAR_CLIMBER_ARRAY);

                Drivetrain.getInstance().setControl(new PositionalControl(alignPose));
                ClimberSubsystem.getInstance().setState(ClimberStates.extend);
                IntakeSubsystem.getInstance().setState(IntakeStates.StoredOff);
                MopSubsystem.getInstance().setState(MopStates.OFF);
                FeederSubsystem.getInstance().setState(FeederStates.OFF);
            }
            case Unclimbed -> {
                ClimberSubsystem.getInstance().setState(ClimberStates.extend);
                manualDrive(robotPose, horizontalVelocity, verticalVelocity, omegaVelocity);
            }
            case Climbing -> {
                ClimberSubsystem.getInstance().setState(ClimberStates.climb);
                MopSubsystem.getInstance().setState(MopStates.OFF);
                FeederSubsystem.getInstance().setState(FeederStates.OFF);
            }
        }

        Drivetrain.getInstance().writePeriodic();
        ClimberSubsystem.getInstance().writePeriodic();
        IntakeSubsystem.getInstance().writePeriodic();
        ShooterSubsystem.getInstance().writePeriodic();
    }

    private void manualDrive(Pose2d robotPose, LinearVelocity horizontalVelocity, LinearVelocity verticalVelocity, AngularVelocity omegaVelocity) {
        if (omegaVelocity.in(Units.RadiansPerSecond) != 0 && Drivetrain.getInstance().getRate().in(Units.RadiansPerSecond) >= DrivetrainConstants.BREAK_YAW_LOCK) {
            Drivetrain.getInstance().setControl(
                new VelocityFOC(
                    horizontalVelocity,
                    verticalVelocity,
                    omegaVelocity
                )
            );
            lockFirstTick = true;
        }else{
            if (lockFirstTick) {
                lockAngle = robotPose.getRotation();
                lockFirstTick = false;
            }
            Drivetrain.getInstance().setControl(
                new YawLockFOC(
                    horizontalVelocity,
                    verticalVelocity,
                    lockAngle
                )
            );
        }
    }

    public void resetGyro() {
        double xPosition= Drivetrain.getInstance().getPose().getX();
        double yPosition = Drivetrain.getInstance().getPose().getY();
        Rotation2d rotation = new Rotation2d();
        if (alliance ==  DriverStation.Alliance.Blue) {
            rotation = Rotation2d.fromRadians(0);
        }else{
            rotation = Rotation2d.fromRadians(Math.PI);
        }
        lockAngle = rotation;
        lockFirstTick = true;
        Drivetrain.getInstance().resetPose(new Pose2d(xPosition, yPosition, rotation));
    }

    public void setAlliance(DriverStation.Alliance alliance) {
        this.alliance = alliance;
        if (alliance == DriverStation.Alliance.Red) {
            CLIMB_LEFT = FieldConstants.BLUE_CLIMB_LEFT;
            CLIMB_RIGHT = FieldConstants.BLUE_CLIMB_RIGHT;

            FERRY_TARGET_LEFT = FieldConstants.BLUE_FERRY_TARGET_LEFT;
            FERRY_TARGET_RIGHT = FieldConstants.BLUE_FERRY_TARGET_RIGHT;
            HOPPER_TARGET = FieldConstants.BLUE_HOPPER;

            NEAR_FERRY_LEFT = FieldConstants.BLUE_NEAR_FERRY_LEFT;
            NEAR_FERRY_RIGHT = FieldConstants.BLUE_NEAR_FERRY_RIGHT;
            NEAR_HOPPER = FieldConstants.BLUE_NEAR_HOPPER;
        }else{
            CLIMB_LEFT = ChoreoAllianceFlipUtil.flip(FieldConstants.BLUE_CLIMB_LEFT);
            CLIMB_RIGHT = ChoreoAllianceFlipUtil.flip(FieldConstants.BLUE_CLIMB_RIGHT);

            FERRY_TARGET_LEFT = ChoreoAllianceFlipUtil.flip(FieldConstants.BLUE_FERRY_TARGET_LEFT);
            FERRY_TARGET_RIGHT = ChoreoAllianceFlipUtil.flip(FieldConstants.BLUE_FERRY_TARGET_RIGHT);
            HOPPER_TARGET = ChoreoAllianceFlipUtil.flip(FieldConstants.BLUE_HOPPER);

            NEAR_FERRY_LEFT = ChoreoAllianceFlipUtil.flip(FieldConstants.BLUE_NEAR_FERRY_LEFT);
            NEAR_FERRY_RIGHT = ChoreoAllianceFlipUtil.flip(FieldConstants.BLUE_NEAR_FERRY_RIGHT);
            NEAR_HOPPER = ChoreoAllianceFlipUtil.flip(FieldConstants.BLUE_NEAR_HOPPER);
        }

        NEAR_TARGET_ARRAY.clear();
        NEAR_TARGET_ARRAY.add(NEAR_FERRY_LEFT);
        NEAR_TARGET_ARRAY.add(NEAR_FERRY_RIGHT);
        NEAR_TARGET_ARRAY.add(NEAR_HOPPER);

        NEAR_CLIMBER_ARRAY.clear();
        NEAR_CLIMBER_ARRAY.add(CLIMB_LEFT);
        NEAR_CLIMBER_ARRAY.add(CLIMB_RIGHT);
    }

    private void setTargetLock(ShooterTargetLockMode targetLock) {
        this.targetLockOn = targetLock;
    }

    public DriverStation.Alliance getAlliance() {
        return alliance;
    }

}
