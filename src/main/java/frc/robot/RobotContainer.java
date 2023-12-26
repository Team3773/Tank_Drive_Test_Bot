// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.BallIntakeCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class RobotContainer {

    // This is where robot subsystems are initialized.
    public final DriveSubsystem m_drive = new DriveSubsystem();
    public final IntakeSubsystem m_feeder = new IntakeSubsystem();

    // Define xbox controller with port mapping.
    private final XboxController controller = new XboxController(0);

    // Constructor for the RobotContainer Class.
    public RobotContainer() {
        // Set Default Command for the drivetrain subsystem. This will be active during
        // teleop mode.
        System.out.println("Creating Commands");
        m_drive.setDefaultCommand(new TeleopDrive(
                m_drive,
                () -> controller.getLeftX(),
                () -> controller.getLeftY()));

        // Set Default Command for the Ball Intake Subsystem
        m_feeder.setDefaultCommand(new BallIntakeCommand(
                m_feeder,
                () -> controller.getLeftBumper(),
                () -> controller.getRightBumper()));
    };
}
