package se.oru.aass.lucia_meta_csp_lecture.exercises;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;

public class Ex1 {
	
	public static void main(String[] args) {
		
		ActivityNetworkSolver temporalSolver = new  ActivityNetworkSolver(0, 100000000);
		
		//Creating the variable "var1", "turtlebot_1" is an annotation of the variable, "move_base" is the value of the variable
		Variable var1 = (SymbolicVariableActivity)temporalSolver.createVariable("turtlebot_1");
		((SymbolicVariableActivity)var1).setSymbolicDomain("move_base");

		//TODO: Creating the variable "var2", "turtlebot_2" is an annotation of the variable and "move_base" is the value of the variable
		Variable var2 = (SymbolicVariableActivity)temporalSolver.createVariable("turtlebot_2");
		((SymbolicVariableActivity)var2).setSymbolicDomain("move_base");
		
		//adding constraints "release" to var1
		AllenIntervalConstraint release = new AllenIntervalConstraint(
				AllenIntervalConstraint.Type.Release, 
				new Bounds(3000,APSPSolver.INF)
				);
		release.setFrom(var1);
		release.setTo(var1);
		temporalSolver.addConstraint(release);
		
		//creating overlap constraint for robot 2
		AllenIntervalConstraint overlap = new AllenIntervalConstraint(
				AllenIntervalConstraint.Type.Overlaps, 
				new Bounds(5000,APSPSolver.INF), 
				new Bounds(1,APSPSolver.INF), 
				new Bounds(1,APSPSolver.INF)
				);
		overlap.setFrom(var1);
		overlap.setTo(var2);
		temporalSolver.addConstraint(overlap);
		
		//TODO: creating Overlaps Constraint between var2 and var1 with an specified bound e.g., 

		
		//#################################################################################
		//visualize
		//#################################################################################
		ConstraintNetwork.draw(temporalSolver.getConstraintNetwork(),"Activity network");

	}

}
