package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.regex.Pattern;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.lights.FlashLEDCommand;
import frc.utils.BlinkinState;
import frc.utils.BlinkinController;

public class LightControl extends SubsystemBase {
    private BlinkinController blinkin;
    private BlinkinState pattern;
    
    public LightControl(int blinkinPort) {
        this.blinkin = new BlinkinController(blinkinPort);
    }

    public void turnOff() { setPattern(BlinkinState.Solid_Colors_Black); }
    
    public void setPattern(BlinkinState pattern) {
        this.pattern = pattern;
    }

    private double getSparkValue() {
        if(pattern == null) {
            return BlinkinState.Color_1_Pattern_Breath_Slow.sparkValue;
        }

        return pattern.sparkValue;
    }

    public boolean isActive() { return pattern != BlinkinState.Solid_Colors_Black; }

    @Override
    public void periodic() {
        double powerOutput = getSparkValue();
        blinkin.setPWM(powerOutput);
    }

    public Command getSetLedCommand(Supplier<BlinkinState> pattern) {
        return runOnce(() -> {
            this.setPattern(pattern.get());
        });
    }

    public Command getSetLedCommand(BlinkinState state) {
        return this.getSetLedCommand(() -> state);
    }
}