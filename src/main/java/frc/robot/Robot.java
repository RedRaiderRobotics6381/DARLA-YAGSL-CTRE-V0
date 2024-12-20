// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.commands.LauncherCmd.IntakeCmd;
import frc.robot.commands.LauncherCmd.OpenHatch;
import frc.robot.subsystems.LEDsSubSystem;
// import frc.robot.subsystems.Vision.ObjectVision;
import frc.robot.subsystems.Launcher;

import java.io.File;
import java.io.IOException;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
@SuppressWarnings("unused")
public class Robot extends TimedRobot
{

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  private Timer disabledTimer;

  public static LEDsSubSystem m_LEDsSubSystem = new LEDsSubSystem(); // Create a new LEDsSubSystem object
  // public static ObjectVision m_ObjectVision = new ObjectVision(m_LEDsSubSystem); // Create a new ObjectVision object

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    // m_ObjectVision = new ObjectVision(m_LEDsSubSystem);
    DriverStation.silenceJoystickConnectionWarning(true); // Silence the joystick connection warning
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
    m_LEDsSubSystem.scanEffect(60, 255, 255);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    aprilTagAlliance();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
    aprilTagAlliance();
    m_robotContainer.initOpenHatch();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    m_robotContainer.spencerButtons();
    // m_ObjectVision.watchForNote();
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      m_LEDsSubSystem.setSolidLED(90, 255, 255);
    } else {
      m_LEDsSubSystem.setSolidLED(0, 255, 255);
    }
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setDriveMode();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
  /**
   * Sets the AprilTag constants based on the alliance color.
   */
  public static void aprilTagAlliance(){
    AprilTagConstants.ampID     = DriverStation.getAlliance().get() == Alliance.Blue ? 6 : 5;
    AprilTagConstants.speakerID = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    AprilTagConstants.stageIDA  = DriverStation.getAlliance().get() == Alliance.Blue ? 14 : 13;
    AprilTagConstants.stageIDB  = DriverStation.getAlliance().get() == Alliance.Blue ? 15 : 12;
    AprilTagConstants.stageIDC  = DriverStation.getAlliance().get() == Alliance.Blue ? 16 : 11;
  }
}
