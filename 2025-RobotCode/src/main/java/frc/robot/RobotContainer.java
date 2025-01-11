package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
 
    /*
    "Imports" subsystems that you make in the subsystem folder to be used for controller actions.
    Make sure you actually import the subsystem in the same manner as we do with the SwerveSubsystem above.
    */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    // Sets the Joystick/Physical Driver Station ports, change port order in Driver Station to the numbers below.
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort); // 0
    private final Joystick driverstationTop = new Joystick(OIConstants.kTopHalfButtonBoardPort); // 1
    private final Joystick driverstationBottom = new Joystick(OIConstants.kBottomHalfButtonBoardPort); // 2
    // private final Joystick debug_secondary = new Joystick(4);

    // Sends a dropdown for us to choose an auto in the Dashboard.
    private final SendableChooser<Command> autoChooser;


    /*
    Assigns raw inputs on whichever joystick you're using into buttons we use to control the robot.
    Feel free to change the names if you decide to change the controller to a non-PS4 controller for clarity sake.
    Check Driver Station for buttonNumbers, they'll be in the USB order tab.
    */
    Trigger X1 = new JoystickButton(driverJoystick, 1);
    Trigger O2 = new JoystickButton(driverJoystick, 2);
    Trigger Square3 = new JoystickButton(driverJoystick, 3);
    Trigger Triangle4 = new JoystickButton(driverJoystick, 4);
    Trigger leftShoulder5 = new JoystickButton(driverJoystick, 5);
    Trigger rightShoulder6 = new JoystickButton(driverJoystick, 6);
    Trigger leftTrigger7 = new JoystickButton(driverJoystick, 7);
    Trigger rightTrigger8 = new JoystickButton(driverJoystick, 8);
    Trigger leftStickPress9 = new JoystickButton(driverJoystick, 9);
    Trigger rightStickPress10 = new JoystickButton(driverJoystick, 10);
    
    Trigger dPadNorth = new POVButton(driverJoystick, 0);
    Trigger dPadSouth = new POVButton(driverJoystick, 180);
    Trigger dPadWest = new POVButton(driverJoystick, 270);
    Trigger dPadEast = new POVButton(driverJoystick, 90);

    Trigger buttonT1 = new JoystickButton(driverstationTop, 1);
    Trigger buttonT2 = new JoystickButton(driverstationTop, 2);
    Trigger buttonT3 = new JoystickButton(driverstationTop, 3);
    Trigger buttonT4 = new JoystickButton(driverstationTop, 4);
    Trigger buttonT5 = new JoystickButton(driverstationTop, 5);
    Trigger buttonT6 = new JoystickButton(driverstationTop, 6);
    Trigger buttonT7 = new JoystickButton(driverstationTop, 7);
    Trigger buttonT8 = new JoystickButton(driverstationTop, 8);
    Trigger buttonT9 = new JoystickButton(driverstationTop, 9);
    Trigger buttonT10 = new JoystickButton(driverstationTop, 10);

    Trigger buttonB1 = new JoystickButton(driverstationBottom, 1);
    Trigger buttonB2 = new JoystickButton(driverstationBottom, 2);
    Trigger buttonB3 = new JoystickButton(driverstationBottom, 3);
    Trigger buttonB4 = new JoystickButton(driverstationBottom, 4);
    Trigger buttonB5 = new JoystickButton(driverstationBottom, 5);
    Trigger buttonB6 = new JoystickButton(driverstationBottom, 6);
    Trigger buttonB7 = new JoystickButton(driverstationBottom, 7);
    Trigger buttonB8 = new JoystickButton(driverstationBottom, 8);
    Trigger buttonB9 = new JoystickButton(driverstationBottom, 9);
    Trigger buttonB10 = new JoystickButton(driverstationBottom, 10);

    // Trigger buttonD7 = new JoystickButton(debug_secondary, 7);
    // Trigger buttonD8 = new JoystickButton(debug_secondary, 8);
    // Trigger buttonD9 = new JoystickButton(debug_secondary, 9);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                /* 
                In teleop, if the robot is moving opposite of the way the joystick is being moved, change one of these
                negatives to a positive depending on how it's inverted.
                */
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
        
        // Use this line to add commands to PathPlanner, make sure to get spelling correct.
        NamedCommands.registerCommand("ResetHeading", new ResetHeading(swerveSubsystem));
        
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Sends a dropdown for us to choose an auto in the Dashboard.
        SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  

    private void configureButtonBindings() {
        /* 
        Used to set all Button Bindings as the name suggests, excluding moving the robot with the joystick,
        which is set with the Command Scheduler.
        */

        X1.onTrue(new ResetHeading(swerveSubsystem));
        O2.onTrue(new ResyncEncoders(swerveSubsystem)); 
        Square3.onTrue(new RedoOffsets(swerveSubsystem));
        // Triangle4.onTrue(new));
        // leftShoulder5.whileTrue(new));
        // rightShoulder6.whileTrue(new));
        // leftTrigger7.whileTrue(new));
        // rightTrigger8.whileTrue(new));
        // leftStickPress9.onTrue(new));
        // rightStickPress10.onTrue(new));
        // dPadNorth.whileTrue(new));
        // dPadEast.whileTrue(new));
        // dPadSouth.whileTrue(new));
        // dPadWest.whileTrue(new));

        // buttonT1.whileTrue(new );
        // buttonT2.whileTrue(new ); 
        // buttonT3.whileTrue(new );
        // buttonT4.onTrue(new);
        // buttonT5.onTrue(new);
        // buttonT6.whileTrue(new);
        // buttonT7.whileTrue(new);
        // buttonT8.whileTrue(new);
        // buttonT9.whileTrue(new);
        // buttonT10.onTrue(new);

        // buttonB1.whileTrue(new);
        // buttonB2.whileTrue(new);
        // buttonB3.whileTrue(new);
        // buttonB4.whileTrue(new);
        // buttonB5.whileTrue(new);
        // buttonB6.onTrue(new);
        // buttonB6.onTrue(new));
        // buttonB7.onTrue(new));
        // buttonB8.onTrue(new));
        // buttonB8.whileTrue(new);
        // buttonB9.whileTrue(new);
        // buttonB10.whileTrue(new);

        // // buttonD7.whileTrue(new);
        // // buttonD8.whileTrue(new);
        // // buttonD9.whileTrue(new);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
  }
    
}
