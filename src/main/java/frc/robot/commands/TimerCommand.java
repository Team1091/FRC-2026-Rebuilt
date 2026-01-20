package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class TimerCommand extends Command {
    private long currentTime = 0;
    private final long delayMilli;

    public TimerCommand(long delayMilli) {
        this.delayMilli = delayMilli;
    }

    @Override
    public void initialize() {
        currentTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - currentTime >= delayMilli;
    }
}