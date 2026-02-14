package org.firstinspires.ftc.teamcode.autonmous.actions;

public class InstantAction extends Action {
    InstantFunction function;
    InstantFunctionThatDoesNotReturnVoid functionThatDoesNotReturnVoid;

    public InstantAction(InstantFunction function) {
        this.function = function;
    }

    public InstantAction(InstantFunctionThatDoesNotReturnVoid function) {
        this.functionThatDoesNotReturnVoid = function;
    }

    @Override
    public boolean run() { // uggos but it works
        if (function != null) {
            function.run();
            return false;
        }
        functionThatDoesNotReturnVoid.run();
        return false;
    }

    public interface InstantFunction {
        void run();
    }

    public interface InstantFunctionThatDoesNotReturnVoid {
        Object run();
    }
}
