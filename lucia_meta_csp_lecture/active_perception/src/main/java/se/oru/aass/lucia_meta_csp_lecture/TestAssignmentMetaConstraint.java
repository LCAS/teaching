package se.oru.aass.lucia_meta_csp_lecture;

import java.util.HashSet;
import java.util.Random;
import java.util.Vector;
import java.util.logging.Level;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint.Type;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.utility.UI.PolygonFrame;
import org.metacsp.utility.logging.MetaCSPLogging;

import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.AssignmentMetaConstraint;
import se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets.LuciaMetaConstraintSolver;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;


public class TestAssignmentMetaConstraint {
	
	public static void main(String[] args) {
		
		//MetaCSPLogging.setLevel(Level.FINE);
		//Symbols represent panels seen by robots
		int numPanels = 6;
		String[] panels = new String[numPanels];
		String[] symbols = new String[numPanels+1];
		for (int i = 0; i < numPanels; i++) {
			panels[i] = "P"+i;
			symbols[i] = "P"+i;
		}
		//Another symbol ("None") represents the fact that a robot sees no panel
		symbols[numPanels] = "None";
		
		LuciaMetaConstraintSolver metaSolver = new LuciaMetaConstraintSolver(0,100000,500,symbols);
		SpatioTemporalSetNetworkSolver groundSolver = (SpatioTemporalSetNetworkSolver)metaSolver.getConstraintSolvers()[0];
		ActivityNetworkSolver groundGroundSolver1 = groundSolver.getActivitySolver();
		GeometricConstraintSolver groundGroundSolver2 = groundSolver.getGeometricSolver();
		SymbolicVariableConstraintSolver groundGroundSolver3 = groundSolver.getSetSolver();
		
		//Vars representing robots and what panels (if any) they see
		Vector<Constraint> initialCondition = new Vector<Constraint>();
		int numRobots = 33;
		Variable[] robots = new Variable[numRobots];
		for (int i = 0; i < numRobots; i++) {
			robots[i] = groundSolver.createVariable("State of Robot"+i);
			SymbolicValueConstraint seesNothing = new SymbolicValueConstraint(SymbolicValueConstraint.Type.VALUEEQUALS);
			robots[i].setMarking(LuciaMetaConstraintSolver.Markings.SUPPORTED);
			seesNothing.setValue("None");
			seesNothing.setFrom(robots[i]);
			seesNothing.setTo(robots[i]);
			initialCondition.add(seesNothing);
			((SpatioTemporalSet)robots[i]).setTask("Observe");
		}
		
		groundSolver.addConstraints(initialCondition.toArray(new Constraint[initialCondition.size()]));
		
		AssignmentMetaConstraint mc1 = new AssignmentMetaConstraint(null, null);
		mc1.setPanels(panels);
		metaSolver.addMetaConstraint(mc1);
		
		metaSolver.backtrack();
		
		System.out.println("Done!");
		
		ConstraintNetwork.draw(groundSolver.getConstraintNetwork());

	}


}
