// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
    
  public class Drivetrain {
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
  
    private static final double kTrackWidth = 0.381 * 2; // meters
    private static final double kWheelRadius = 0.0508; // meters
    private static final int kEncoderResolution = 4096;
  
  private final MotorController m_leftFront = new PWMVictorSPX(4);
  private final MotorController m_rightFront = new PWMTalonSRX(2);
  private final MotorController m_leftBack = new PWMVictorSPX(3);
  private final MotorController m_rightBack = new PWMTalonSRX(1);
  private final MotorController m_leftIntake = new PWMTalonSRX(5);
  private final MotorController m_rightIntake = new PWMTalonSRX(6);


  private final MotorControllerGroup m_leftGroup =
  new MotorControllerGroup(m_leftFront, m_leftBack);
  private final MotorControllerGroup m_rightGroup =
  new MotorControllerGroup(m_rightFront, m_rightBack);
  private final MotorControllerGroup m_intakeGroup =
  new MotorControllerGroup(m_leftIntake, m_rightIntake);
  
  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);
  
  private final AnalogGyro m_gyro = new AnalogGyro(0);
    
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);
      public Drivetrain() {
        m_gyro.reset();
    
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightGroup.setInverted(true);
      
        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        
        

        m_leftEncoder.reset();
        m_rightEncoder.reset();
    
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
      }
    
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightGroup.setInverted(true);
    m_rightIntake.setInverted(true);
    m_myRobot = new DifferentialDrive(m_leftGroup, m_rightGroup);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    
  }
 
  private PIDController m_feedforward;
  private double trackWidthMeters;
    
  @param //speeds The desired wheel speeds.
  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public ChassisSpeeds toChassisSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    return new ChassisSpeeds(
        (wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond) / 2,
        0,
        (wheelSpeeds.rightMetersPerSecond - wheelSpeeds.leftMetersPerSecond) / trackWidthMeters);
  }
  public void DifferentialDriveKinematics(double trackWidthMeters) {
    this.trackWidthMeters = trackWidthMeters;
    MathSharedStore.reportUsage(MathUsageId.kKinematics_DifferentialDrive, 1);
  }
  public void drive(double xSpeed, double rot) {
    DifferentialDriveKinematics m_kinematics;
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 1.0, rot));
    setSpeeds(wheelSpeeds);
  }
  
  
  public void teleopPeriodic() {
    m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
    }
    
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
      final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
      final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
  
      AnalogGyro m_leftEncoder;
      final double leftOutput =
          m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
      AnalogGyro m_rightEncoder;
      final double rightOutput =
          m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
      m_leftGroup.setVoltage(leftOutput + leftFeedforward);
      m_rightGroup.setVoltage(rightOutput + rightFeedforward);
    }
  }
}
