// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot  
{
  XboxController Controller = new XboxController(0);
  VictorSP FrontRightDriveMotor = new VictorSP(2);
  VictorSP FrontLeftDriveMotor = new VictorSP(0);
  VictorSP BackRightDriveMotor = new VictorSP(3);
  VictorSP BackLeftDriveMotor = new VictorSP(1);
  double TimeSinceStartAtAutoStart;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    FrontRightDriveMotor.setInverted(true);
    //BackRightDriveMotor.setInverted(true);
    FrontLeftDriveMotor.setInverted(true);
    BackLeftDriveMotor.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */

  public void autonomousInit() 
  {
    TimeSinceStartAtAutoStart=Timer.getFPGATimestamp();

    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    if(Timer.getFPGATimestamp() - TimeSinceStartAtAutoStart < 2)
    {
      
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() 
  {
    FrontRightDriveMotor.set(Controller.getLeftY()+Controller.getLeftX());
    BackRightDriveMotor.set(Controller.getLeftY()+Controller.getLeftX());
    FrontLeftDriveMotor.set(Controller.getLeftY()-Controller.getLeftX());
    BackLeftDriveMotor.set(Controller.getLeftY()-Controller.getLeftX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}