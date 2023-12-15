// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class DriveSubsystem extends SubsystemBase {

  //   public static final double kMaxSpeed = 3.0; // meters per second
  //   public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
  
  //   private static final double kTrackWidth = 0.381 * 2; // meters
  //   private static final double kWheelRadius = 0.0508; // meters
  //   private static final int kEncoderResolution = 4096;
  /** Creates a new DriveTrain. */
  private final MotorController m_leftFront = new PWMVictorSPX(4);
  private final MotorController m_rightFront = new PWMTalonSRX(2);
  private final MotorController m_leftBack = new PWMVictorSPX(3);
  private final MotorController m_rightBack = new PWMTalonSRX(1);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftFront, m_leftBack);  
  private final MotorControllerGroup m_rightGroup= new MotorControllerGroup(m_rightFront, m_rightBack);
  
  private DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  public DriveSubsystem() {
    m_leftGroup.setInverted(false);
    m_rightGroup.setInverted(true);   
  }


  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed * -1, rotation * -1);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
