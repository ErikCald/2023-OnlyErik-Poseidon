package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerveCommand extends CommandBase {
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  public TeleopSwerveCommand(CommandXboxController driver) {
    this(
      () -> -driver.getRawAxis(Config.JoystickConfig.TRANSLATION_AXIS),
      () -> -driver.getRawAxis(Config.JoystickConfig.STRAFE_AXIS),
      () -> -driver.getRawAxis(Config.JoystickConfig.ROTATION_AXIS));
  }
  public TeleopSwerveCommand(
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;

    addRequirements(SwerveSubsystem.getInstance());
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Config.Swerve.JOYSTICK_DEADBAND);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Config.Swerve.JOYSTICK_DEADBAND);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Config.Swerve.JOYSTICK_DEADBAND);

    translationVal *= Config.Swerve.TELEOP_SPEED;
    strafeVal *= Config.Swerve.TELEOP_SPEED;
    rotationVal *= Config.Swerve.TELEOP_ANGULAR_SPEED;

    /* Drive */
    SwerveSubsystem.getInstance().drive(
        translationVal, strafeVal,
        rotationVal,
        true,
        true);
  }
}
