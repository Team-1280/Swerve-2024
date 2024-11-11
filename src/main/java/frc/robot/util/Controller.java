package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controller {
    static Trigger no_op = new Trigger(() -> false);

    default double getLeftX() {
        return 0;
    }

    default double getLeftY() {
        return 0;
    }

    default double getRightX() {
        return 0;
    }

    default double getRightY() {
        return 0;
    }

    default Trigger resetHeading() {
        return no_op;
    }
}
