package org.firstinspires.ftc.teamcode.Instructions;

import java.util.ArrayList;

public class InstructionExecutor {
    private ArrayList<Instruction> instructions;

    public void addInstruction(Instruction newInstruction) {
        instructions.add(newInstruction);
    }

    public void clearInstructions() {
        instructions.clear();
    }

    public void executeInstructions() {
        for (Instruction instruction : instructions) {
            instruction.run();
        }

        clearInstructions();
    }

    public Instruction getNextInstruction() {
        Instruction nextInstruction = instructions.get(0);
        instructions.remove(0);
        return nextInstruction;
    }
}
