package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Trigger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;

public class ControllerCondition {
    private final List<Trigger> triggers = new ArrayList<>();
    private final CustomController controller;

    private final Trigger checkTriggers = new Trigger(() -> {
        for (Trigger trigger : triggers) if (!trigger.get()) return false;
        return true;
    });

    public ControllerCondition(CustomController controller, Trigger ... triggers) {
        this.controller = controller;
        Collections.addAll(this.triggers, triggers);
    }

    public ControllerCondition and(Trigger additionalTrigger) {
        triggers.add(additionalTrigger);
        return this;
    }

    public ControllerCondition and(BooleanSupplier booleanSupplier) {
        return and(new Trigger(booleanSupplier));
    }

    public CustomController whenActive(Command command) {
        checkTriggers.whenActive(command);
        return controller;
    }

    public CustomController toggleWhenActive(Command firstCommand, Command secondCommand) {
        checkTriggers.toggleWhenActive(firstCommand, secondCommand);
        return controller;
    }
}
