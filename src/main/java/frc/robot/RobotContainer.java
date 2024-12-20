// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LauncherCmd.IntakeCmd;
import frc.robot.commands.LauncherCmd.OpenHatch;
import frc.robot.commands.LauncherCmd.ScoreAmpCmd;
import frc.robot.commands.LauncherCmd.ScoreSpeakerCmd;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdvHdgAim;
import frc.robot.subsystems.Launcher;
// import frc.robot.subsystems.Vision.FiducialVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  public Launcher m_launcher = new Launcher();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) *
                                                        Constants.DrivebaseConstants.Max_Speed_Multiplier,
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) *
                                                        Constants.DrivebaseConstants.Max_Speed_Multiplier,
    () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) *
                                                          Constants.DrivebaseConstants.Max_Speed_Multiplier,
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) *
                                                          Constants.DrivebaseConstants.Max_Speed_Multiplier,
    () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // rotation control is selectable between direct angle and angular velocity
  // left stick controls translation
  // in one mode the right stick controls the rotational velocity 
  // in the other mode the right stick controls the desired angle NOT angular rotation
  // also in this mode the POV buttons are used to quickly face a direction
  // and a button will yaw the robot towards a target.
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  Command AbsoluteDriveAdvHdgAim = new AbsoluteDriveAdvHdgAim(drivebase,
                                                                    () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                  OperatorConstants.LEFT_Y_DEADBAND) *
                                                                                                  DrivebaseConstants.Max_Speed_Multiplier,
                                                                    () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                  OperatorConstants.LEFT_X_DEADBAND) *
                                                                                                  DrivebaseConstants.Max_Speed_Multiplier,
                                                                    () -> MathUtil.applyDeadband(driverXbox.getRightX(),OperatorConstants.LEFT_X_DEADBAND),
                                                                    () -> MathUtil.applyDeadband(driverXbox.getRightY(),OperatorConstants.LEFT_Y_DEADBAND),
                                                                    driverXbox.povUp(),
                                                                    driverXbox.povDown(),
                                                                    driverXbox.povRight(),
                                                                    driverXbox.povLeft(),
                                                                    driverXbox.b(),
                                                                    driverXbox.leftStick());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
    //Button 1 is "A" on xbox controller
    //Button 2 is "B" on xbox controller
    //Button 3 is "X" on xbox controller  
    //Button 4 is "Y" on xbox controller
    //Button 5 is "Left Bumper" on xbox controller
    //Button 6 is "Right Bumper" on xbox controller
    //Button 7 is "Back" on xbox controller
    //Button 8 is "Start" on xbox controller
    //Button 9 is "Left Joystick" on xbox controller
    //Button 10 is "Right Joystick" on xbox controller
    //Axis 0 is left joystick x side to side
    //Axis 1 is left joystick y forward and back
    //Axis 2 is left trigger 
    //Axis 3 is right trigger
    //Axis 4 is right joystick x side to side
    //Axis 5 is right joystick y forward and back

   private void configureBindings()
  {
    if (DriverStation.isTest())
    {
      driverXbox.y().whileTrue(drivebase.sysIdAngleMotorCommand());
      driverXbox.x().whileTrue(drivebase.sysIdDriveMotorCommand());
      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.b().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.leftBumper().onTrue(Commands.none());
      // driverXbox.rightBumper().onTrue(Commands.none());
      // drivebase.setDefaultCommand(
      //     !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
      drivebase.setDefaultCommand(AbsoluteDriveAdvHdgAim);
    } else
    {
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.a().onTrue(new IntakeCmd(m_launcher));
      driverXbox.x().onTrue(new ScoreAmpCmd(m_launcher));
      driverXbox.y().onTrue(new ScoreSpeakerCmd(m_launcher));
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     Commands.deferredProxy(() -> drivebase.driveToPose(
      //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                           ));
      // driverXbox.b().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
      //                         FiducialVision.getAprilTagPose(AprilTagConstants.speakerID,
      //                                                       new Transform2d(1.7, 0,
      //                                                       Rotation2d.fromDegrees(0))))));
      // driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
      // driverXbox.start().whileTrue(Commands.none());
      // driverXbox.back().whileTrue(Commands.none());
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.rightBumper().onTrue(Commands.none());
      // drivebase.setDefaultCommand(
      //     !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
      drivebase.setDefaultCommand(AbsoluteDriveAdvHdgAim);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  /**
   * Sets the speed multiplier for the drivebase based on the state of the right and left bumpers on the driver's Xbox controller.
   * If both bumpers are pressed, the speed multiplier is set to 1 (HighSpd).
   * If either bumper is pressed, the speed multiplier is set to 0.75 (MedSpd).
   * If neither bumper is pressed, the speed multiplier is set to 0.50 (LowSpd).
   */
  public void spencerButtons(){

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("HighSpd");
      DrivebaseConstants.Max_Speed_Multiplier = 1;
    }

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == false ||
        driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("MedSpd");
      DrivebaseConstants.Max_Speed_Multiplier = .875;
    }

    if (driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == false){
      //System.out.println("LowSpd");
      DrivebaseConstants.Max_Speed_Multiplier = .75;
    }
  }
  public void initOpenHatch(){
    new OpenHatch(m_launcher).schedule();
  }
}
