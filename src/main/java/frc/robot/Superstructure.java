package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.IO.Controls;
import frc.robot.IO.IO;
import frc.robot.command.PathFollower;
import frc.robot.subsystem.drivetrain.Drivetrain;
import frc.robot.subsystem.drivetrain.DrivetrainConstants;
import frc.robot.subsystem.drivetrain.control.VelocityFOC;
import frc.robot.subsystem.drivetrain.control.YawLockFOC;
import frc.robot.subsystem.feeder.FeederStates;
import frc.robot.subsystem.feeder.FeederSubsystem;
import frc.robot.subsystem.intake.IntakeStates;
import frc.robot.subsystem.intake.IntakeSubsystem;
import frc.robot.subsystem.mop.MopStates;
import frc.robot.subsystem.mop.MopSubsystem;
import frc.robot.subsystem.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Optional;

public class Superstructure {
    private static Superstructure INSTANCE = new Superstructure();
    public static Superstructure getInstance() {
        return INSTANCE;
    }

    private RobotStates robotState;

    private DriverStation.Alliance alliance;
    private Pose2d FERRY_TARGET_LEFT = new Pose2d();
    private Pose2d FERRY_TARGET_RIGHT = new Pose2d();

    private Pose2d HOPPER_TARGET = new Pose2d();
    private Pose2d NEAR_FERRY_LEFT = new Pose2d();
    private Pose2d NEAR_FERRY_RIGHT = new Pose2d();

    private Pose2d NEAR_HOPPER = new Pose2d();

    private Pose2d targetPose = new Pose2d();

    private final ArrayList<Pose2d> NEAR_TARGET_ARRAY = new ArrayList<>();

    private Pose2d shooterTarget = new Pose2d();
    private Rotation2d lockAngle;

    private boolean lockFirstTick;

    private enum Paths {
        Left,
        Right,
        Center;
    }
    private final SendableChooser<Paths> autoChooser;
    private Paths autoPath = Paths.Left;
    private Command autoCommand = new InstantCommand(() -> System.out.println("No Autos"));

    public void setRobotState(RobotStates robotState) {
        this.robotState = robotState;
    }

    private enum ShooterTargetLockMode{
        AutoAlign,
        None
    }


    private final SendableChooser<ShooterTargetLockMode> lockShooterChooser;
    private ShooterTargetLockMode targetLockOn = ShooterTargetLockMode.AutoAlign;

    private Superstructure(){
        setRobotState(RobotStates.Default);
        ChoreoAllianceFlipUtil.setYear(2026);
        setAlliance(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));

        lockShooterChooser = new SendableChooser<>();
        SmartDashboard.putData("Lock Shooter Chooser", lockShooterChooser);
        lockShooterChooser.setDefaultOption("Auto Align On", ShooterTargetLockMode.AutoAlign);
        lockShooterChooser.addOption("Auto Align Off", ShooterTargetLockMode.None);
        lockShooterChooser.onChange(this::setTargetLock);

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Left", Paths.Left);
        autoChooser.addOption("Right", Paths.Right);
        autoChooser.addOption("Center", Paths.Center);
        autoChooser.onChange(this::generateAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        autoPath = Paths.Left;

        lockFirstTick = false;
        lockAngle = new Rotation2d();
    }

    public void readPeriodic() {
        Drivetrain.getInstance().readPeriodic();
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
        ShooterSubsystem.getInstance().setTarget(robotPose, shooterTarget, (shooterTarget == FERRY_TARGET_LEFT || shooterTarget == FERRY_TARGET_RIGHT));
        targetPose = shooterTarget;

        Logger.recordOutput("ShooterTarget", shooterTarget);
        Logger.recordOutput("Distance", Math.sqrt(targetPose.relativeTo(Drivetrain.getInstance().getPose()).getX() * targetPose.relativeTo(Drivetrain.getInstance().getPose()).getX() + targetPose.relativeTo(Drivetrain.getInstance().getPose()).getY() * targetPose.relativeTo(Drivetrain.getInstance().getPose()).getY()));

        Rotation2d targetLock = new Rotation2d(
            robotPose.getX() - shooterTarget.getX() - .098,
            robotPose.getY() - shooterTarget.getY() - .212
        );

        // Drivetrain Input
        double horizontalInput = IO.getJoystick(Controls.horizontalVelocity).getAsDouble();
        double verticalInput = IO.getJoystick(Controls.verticalVelocity).getAsDouble();
        double omegaInput = -IO.getJoystick(Controls.omega).getAsDouble();

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
        AngularVelocity omegaVelocity = AngularVelocity.ofBaseUnits(omegaInput * DrivetrainConstants.MAX_ANGULAR_VELOCITY, Units.RadiansPerSecond);

        switch (robotState) {
            case Default -> {
                manualDrive(robotPose, horizontalVelocity, verticalVelocity, omegaVelocity);
            }
            case Shooting -> {
                if (targetLockOn == ShooterTargetLockMode.AutoAlign) {
                    targetPose = new Pose2d(Drivetrain.getInstance().getPose().getX(), Drivetrain.getInstance().getPose().getY(), targetLock.plus(Rotation2d.fromDegrees(90)));
                    Drivetrain.getInstance().setControl(
                        new YawLockFOC(
                            horizontalVelocity,
                            verticalVelocity,
                            targetPose.getRotation()
                        )
                    );
                }else{
                    manualDrive(robotPose, horizontalVelocity, verticalVelocity, omegaVelocity);
                }

                // Auto shoot, if we decide to try it out
//
                if (Drivetrain.getInstance().getControl().atSetpoint()) {
                    FeederSubsystem.getInstance().setState(FeederStates.FEED);
                    MopSubsystem.getInstance().setState(MopStates.FEED);
                    if (IntakeSubsystem.getInstance().getState() == IntakeStates.Deployed){
                        IntakeSubsystem.getInstance().setState(IntakeStates.Deployed);
                    }else{
                        IntakeSubsystem.getInstance().setState(IntakeStates.StoredFwd);
                    }
                }else{
                    MopSubsystem.getInstance().setState(MopStates.OFF);
                    FeederSubsystem.getInstance().setState(FeederStates.OFF);
                    if (IntakeSubsystem.getInstance().getState() == IntakeStates.Deployed){
                        IntakeSubsystem.getInstance().setState(IntakeStates.Deployed);
                    }else{
                        IntakeSubsystem.getInstance().setState(IntakeStates.StoredFwd);
                    }
                }
            }
            case Auto -> {

            }
        }

        Drivetrain.getInstance().writePeriodic();
        IntakeSubsystem.getInstance().writePeriodic();
        ShooterSubsystem.getInstance().writePeriodic();
        loggingMechanism();

        Logger.recordOutput("Robot State", robotState);
        Logger.recordOutput("Target Pose", targetPose);
    }

    public void setPose(Pose2d pose) {
        Drivetrain.getInstance().resetPose(pose);
    }

    public Command getAutonomousCommand() {
        return autoCommand;
    }
    private void manualDrive(Pose2d robotPose, LinearVelocity horizontalVelocity, LinearVelocity verticalVelocity, AngularVelocity omegaVelocity) {
        if (omegaVelocity.in(Units.RadiansPerSecond) == 0 && Drivetrain.getInstance().getRate().in(Units.RadiansPerSecond) >= DrivetrainConstants.BREAK_YAW_LOCK) {
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
        }else{
            Drivetrain.getInstance().setControl(
                new VelocityFOC(
                    horizontalVelocity,
                    verticalVelocity,
                    omegaVelocity
                )
            );
            lockFirstTick = true;
        }
    }

    public void resetGyro() {
        double xPosition= Drivetrain.getInstance().getPose().getX();
        double yPosition = Drivetrain.getInstance().getPose().getY();
        Rotation2d rotation = new Rotation2d();
        if (getAlliance() ==  DriverStation.Alliance.Blue) {
            rotation = Rotation2d.fromRadians(0);
        }else{
            rotation = Rotation2d.fromRadians(Math.PI);
        }
        lockAngle = rotation;
        lockFirstTick = true;
        Drivetrain.getInstance().resetPose(new Pose2d(xPosition, yPosition, rotation));
    }

    private void loggingMechanism() {
        Pose3d intakePose = new Pose3d(0.2790, 0, 0.2414524,new Rotation3d());
        if (IntakeSubsystem.getInstance().getState() == IntakeStates.Deployed || IntakeSubsystem.getInstance().getState() == IntakeStates.DeployedRevs){
            intakePose = new Pose3d(0.259500, 0, 0.2454524,new Rotation3d(0,0,0));
        }else{
            intakePose = new Pose3d(0.259500, 0, 0.2454524,new Rotation3d(0,Math.toRadians(-98),0));
        }
        Logger.recordOutput("Mech", intakePose);
    }

    public void setAlliance(DriverStation.Alliance alliance) {

        this.alliance = alliance;
        if (alliance == DriverStation.Alliance.Blue) {

            FERRY_TARGET_LEFT = FieldConstants.BLUE_FERRY_TARGET_LEFT;
            FERRY_TARGET_RIGHT = FieldConstants.BLUE_FERRY_TARGET_RIGHT;
            HOPPER_TARGET = FieldConstants.BLUE_HOPPER;

            NEAR_FERRY_LEFT = FieldConstants.BLUE_NEAR_FERRY_LEFT;
            NEAR_FERRY_RIGHT = FieldConstants.BLUE_NEAR_FERRY_RIGHT;
            NEAR_HOPPER = FieldConstants.BLUE_NEAR_HOPPER;
        }else{
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

        generateAuto(autoPath);
    }

    private Trajectory<SwerveSample> loadTrajectory(String trajectoryName) {
        Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(trajectoryName);
        if (trajectory.isPresent()) {
            System.out.println("Trajectory loaded: " + trajectoryName);
            return trajectory.get();
        }else{
            System.out.println("Trajectory not found: " + trajectoryName);
        }
        return null;
    }

    private void generateAuto(Paths path) {
        autoPath = path;
        if (path == Paths.Left ||  path == Paths.Right) {
            Trajectory<SwerveSample> grabMiddle = null;
            Pose2d handoffPose = new Pose2d();
            switch (path) {
                case Left:
                    grabMiddle = loadTrajectory("leftIntake");
                    break;
                case Right:
                    grabMiddle = loadTrajectory("rightIntake");
                    break;
            }
            if (getAlliance() == DriverStation.Alliance.Red) {
                grabMiddle = grabMiddle.flipped();
            }
            handoffPose = grabMiddle.getInitialPose(false).get();
            setPose(handoffPose);
            autoCommand = new SequentialCommandGroup(
                new InstantCommand(() -> IntakeSubsystem.getInstance().setState(IntakeStates.Deployed)),
                new InstantCommand(() -> setRobotState(RobotStates.Shooting)),
                new WaitCommand(2.5),
                new InstantCommand(() -> {
                    setRobotState(RobotStates.Auto);
                    MopSubsystem.getInstance().setState(MopStates.OFF);
                    FeederSubsystem.getInstance().setState(FeederStates.OFF);
                    IntakeSubsystem.getInstance().setState(IntakeStates.Deployed);
                }),
                new PathFollower(grabMiddle),
                new InstantCommand(() -> setRobotState(RobotStates.Shooting)),
                new WaitCommand(4),
                new InstantCommand(() -> {
                    MopSubsystem.getInstance().setState(MopStates.OFF);
                    FeederSubsystem.getInstance().setState(FeederStates.OFF);
                    Superstructure.getInstance().setRobotState(RobotStates.Auto);
                    IntakeSubsystem.getInstance().setState(IntakeStates.Deployed);
                }),
                new PathFollower(grabMiddle)
            );
        }else{
            Pose2d handoffPose = new Pose2d();
            Trajectory<SwerveSample> part1 = loadTrajectory("middlePart1");
            Trajectory<SwerveSample> part2 = loadTrajectory("middlePart2");
            if (getAlliance() == DriverStation.Alliance.Red) {
                part1 = part1.flipped();
                part2 = part2.flipped();
            }
            handoffPose = part1.getInitialPose(false).get();
            setPose(handoffPose);
            autoCommand = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Superstructure.getInstance().setRobotState(RobotStates.Shooting);
                }),
                new WaitCommand(4),
                new InstantCommand(() -> {
                    MopSubsystem.getInstance().setState(MopStates.OFF);
                    FeederSubsystem.getInstance().setState(FeederStates.OFF);
                    Superstructure.getInstance().setRobotState(RobotStates.Auto);
                    IntakeSubsystem.getInstance().setState(IntakeStates.Deployed);
                }),
                new ParallelDeadlineGroup(
                        new WaitCommand(7),
                        new PathFollower(part1.getSplit(0).get())
                ),
                new PathFollower(part1.getSplit(1).get()),
                new InstantCommand(() -> {
                    Superstructure.getInstance().setRobotState(RobotStates.Shooting);
                }),
                new WaitCommand(4),
                new InstantCommand(() -> {
                    MopSubsystem.getInstance().setState(MopStates.OFF);
                    FeederSubsystem.getInstance().setState(FeederStates.OFF);
                    Superstructure.getInstance().setRobotState(RobotStates.Auto);
                    IntakeSubsystem.getInstance().setState(IntakeStates.Deployed);
                }),
                new PathFollower(part2),
                new InstantCommand(() -> {
                    Superstructure.getInstance().setRobotState(RobotStates.Auto);
                })
            );
//
//            Pose2d handoff = new Pose2d();
//            Trajectory<SwerveSample> autosPath = loadTrajectory("pobots");
//            if (getAlliance() == DriverStation.Alliance.Red) {
//                autosPath = autosPath.flipped();
//            }
//          handoff = autosPath.getInitialPose(false).get();
//            setPose(handoff);
//            autoCommand = new SequentialCommandGroup(
//                    new PathFollower(autosPath),
//                    new InstantCommand(() -> {
//                        Superstructure.getInstance().setRobotState(RobotStates.Shooting);
//                    }),
//                    new WaitCommand(4)
//            );

        }
    }
    public void disablePeriodic() {
        if (alliance != DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            generateAuto(autoPath);
            setAlliance(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
        }
    }
    public Command getAuto() {
        return autoCommand;
    }
    private void setTargetLock(ShooterTargetLockMode targetLock) {
        this.targetLockOn = targetLock;
    }

    public DriverStation.Alliance getAlliance() {
        return alliance;
    }

}
