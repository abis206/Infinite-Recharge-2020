package frc.team8051.commands.drivebase;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team8051.Robot;
import frc.team8051.services.OI;

public class TestDrive extends Command {
    private OI oi;
    private DifferentialDrive differentialDrive;
    public TestDrive(OI oi, DifferentialDrive differentialDrive) {
        this.oi = oi;
        this.differentialDrive = differentialDrive;
    }

    public TestDrive() {
        this(Robot.getInstance().getOI(), Robot.getInstance().getDifferentialDrive());
    }

    @Override
    protected void execute() {
        double limit = 0.5;
        double r = -oi.getRightYAxis(oi.getOpertorJoystick());
        double l = -oi.getLeftYAxis(oi.getOpertorJoystick());
        differentialDrive.tankDrive(r*limit, l*limit);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {

    }
}
