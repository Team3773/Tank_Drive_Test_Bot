// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveSubsystem extends SubsystemBase {

  // public static final double kMaxSpeed = 3.0; // meters per second
  // public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation
  // per second

  // private static final double kTrackWidth = 0.381 * 2; // meters
  // private static final double kWheelRadius = 0.0508; // meters
  // private static final int kEncoderResolution = 4096;
  /** Creates a new DriveTrain. */
  private final MotorController m_leftFront = new WPI_VictorSPX(4);
  private final MotorController m_rightFront = new WPI_TalonSRX(2);
  private final MotorController m_leftBack = new WPI_VictorSPX(3);
  private final MotorController m_rightBack = new WPI_TalonSRX(1);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftFront, m_leftBack);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightFront, m_rightBack);

  private DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  public DriveSubsystem() {
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }
}
