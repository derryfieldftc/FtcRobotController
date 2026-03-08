package org.firstinspires.ftc.teamcode.autonmous.actions;

/**
 * This action is meant as a wrapper for normal functions. Its basically just
 * syntantic sugar
 */
public class InstantAction extends Action {
	InstantFunction function;
	InstantFunctionThatDoesNotReturnVoid functionThatDoesNotReturnVoid;

	// This has two instantiators, one for a function that returns void (this one)
	public InstantAction(InstantFunction function) {
		this.function = function;
	}

	/**
	 * This is used for any function which returns a value, realize you cannot use
	 * the return value of said function, you just run it
	 */
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

	// This is used to appease the java compiler, as for SOME REASON VOID AND OBJECT
	// ARE NOT COERCIBLE TO EACHOTHER
	public interface InstantFunction {
		void run();
	}

	public interface InstantFunctionThatDoesNotReturnVoid {
		Object run();
	}
}
