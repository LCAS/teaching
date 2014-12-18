package se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Random;
import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.framework.VariableOrderingH;
import org.metacsp.framework.meta.FocusConstraint;
import org.metacsp.framework.meta.MetaConstraint;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicVariable;
import org.metacsp.multi.symbols.SymbolicValueConstraint.Type;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.metacsp.utility.Permutation;

import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;
import se.oru.aass.lucia_meta_csp_lecture.util.RobotFactory;

public class AssignmentMetaConstraint extends MetaConstraint {

	private static final long serialVersionUID = -7898167521782702985L;
	
	private String[] panels;
	
	public AssignmentMetaConstraint(VariableOrderingH varOH, ValueOrderingH valOH) {
		super(varOH, valOH);
	}
	
	public void setPanels(String[] panels) {
		this.panels = panels;
	}
	
	@Override
	public ConstraintNetwork[] getMetaVariables() {

		//If no focus, create one - the first time around,
		//this should contain all variables, as the only thing that
		//appears in the network is the initial Observes of the
		//robots
		if (this.metaCS.getCurrentFocusConstraint() == null) {
			this.metaCS.setFocus(this.getGroundSolver().getVariables());
		}
		
		//Number of variables in focus is always = number of robots
		Variable[] varsInFocus = this.metaCS.getFocused();
		Vector<Variable> currentVarsInFocus = new Vector<Variable>();
		
		//Filter vars that are in focus but not sensor readings (i.e., just expectations) 
		for (Variable var : varsInFocus) {
			if (((LuciaMetaConstraintSolver)this.metaCS).isSensorReading((SpatioTemporalSet)var)) {
				currentVarsInFocus.add(var);
			}
		}
		
		int numRobots = currentVarsInFocus.size();
		
		//Return "no conflict" if there aren't enough robots to cover the panels
		//(and reset focus not null, so we do it all next time...)
		if (numRobots < this.panels.length) {
			if (varsInFocus.length == numRobots) {
				this.metaCS.setCurrentFocusConstraint(null);
				System.out.println("NOT ENOUGH ROBOTS... WAITING");
			}
			else {
				System.out.println("NOT ALL ROBOTS FINISHED... WAITING");
			}
			return null;
		}
		
		boolean thereIsAnUnseenPanel = false;
		for (int i = 0; i < panels.length; i++) {
			boolean unseen = true;
			System.out.println("Looking for robot that sees " + panels[i]);
			for (Variable var : currentVarsInFocus) {
				SymbolicVariable sv = ((SpatioTemporalSet)var).getSet();
				for (String s : sv.getSymbols()) {
					if (s.equals(panels[i])) {
						unseen = false;
						System.out.println("Yes: " + var.getComponent() + " sees " + panels[i]);
						break;
					}
				}
			}
			if (unseen) {
				thereIsAnUnseenPanel = true;
				System.out.println("Could not find a robot that sees " + panels[i]);
				break;
			}
		}
		
		if (!thereIsAnUnseenPanel) return null;

		System.out.println("There are unseen panels...");
		//randomly get SpatioTemporalSet[] availableRobots
		Random rand = new Random(1234431);
		HashSet<Variable> chosenRobots = new HashSet<Variable>();
		ConstraintNetwork ret = new ConstraintNetwork(null);
		for (int i = 0; i < panels.length; i++)
			while (!chosenRobots.add(currentVarsInFocus.get(rand.nextInt(numRobots)))) {}
		for (Variable var : chosenRobots) ret.addVariable(var);
		return new ConstraintNetwork[] {ret};
	}

	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {
		// Input: SpatioTemporaSet[] selectedRobots
		// create new SpatioTemporaSet[] selectedRobotsFuture
		Vector<Variable> newVars = new Vector<Variable>();
		Vector<Constraint> newCons = new Vector<Constraint>();
		for (Variable var : metaVariable.getConstraintNetwork().getVariables()) {
			Variable newVar = RobotFactory.createSpatioTemporalSetVariable(var.getComponent(), new Vec2(0.0f,0.0f), 0.0f, this.getGroundSolver());
			newVar.setMarking(LuciaMetaConstraintSolver.Markings.UNSUPPORTED);
			newVars.add(newVar);
			((SpatioTemporalSet)newVar).setTask("Observe");
		}
		
		//Force every chosen robot to see one of the panels (w/o deciding which panel)		
		for (Variable robot : newVars) {
			SymbolicValueConstraint con = new SymbolicValueConstraint(Type.VALUESUBSET);
			con.setValue(panels);
			con.setFrom(robot);
			con.setTo(robot);
			newCons.add(con);
		}

		//Force all chosen robots to see a different panel
		SymbolicValueConstraint con = new SymbolicValueConstraint(Type.DIFFERENT);
		con.setScope(newVars.toArray(new Variable[newVars.size()]));
		newCons.add(con);

		// ConstraintNetwork[] metaValues = unary value constraints modeling one combination
		Vector<ConstraintNetwork> metaValues = new Vector<ConstraintNetwork>();
		Permutation p = new Permutation(newVars.size(),newVars.size());
	    while (p.hasNext()) {
	      int[] a = p.next();
	      ConstraintNetwork cn = new ConstraintNetwork(null);
	      for (int i = 0; i < a.length; i++) {
	    	  Variable var = newVars.elementAt(a[i]);
	    	  SymbolicValueConstraint chooseValue = new SymbolicValueConstraint(SymbolicValueConstraint.Type.VALUEEQUALS);
	    	  chooseValue.setValue(panels[i]);
	    	  chooseValue.setFrom(var);
	    	  chooseValue.setTo(var);
	    	  AllenIntervalConstraint durationObserve = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3000,APSPSolver.INF));
	    	  durationObserve.setFrom(var);
	    	  durationObserve.setTo(var);
	    	  cn.addConstraints(chooseValue,durationObserve);
	      }
	      for (Constraint newCon : newCons) cn.addConstraint(newCon);
	      cn.setAnnotation(this);
	      metaValues.add(cn);
	    }

	    //Create the new current situation 
		this.metaCS.setFocus(newVars.toArray(new Variable[newVars.size()]));
//		System.out.println("VARS IN FOCUS ARE NOW: " + this.metaCS.getFocused().length);
		return metaValues.toArray(new ConstraintNetwork[metaValues.size()]);
	}

	@Override
	public void markResolvedSub(MetaVariable metaVariable, ConstraintNetwork metaValue) {
		// TODO Auto-generated method stub
	}

	@Override
	public void draw(ConstraintNetwork network) {
		// TODO Auto-generated method stub
	}

	@Override
	public ConstraintSolver getGroundSolver() {
		return this.metaCS.getConstraintSolvers()[0];
	}

	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return this.getClass().getSimpleName();
	}

	@Override
	public String getEdgeLabel() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Object clone() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean isEquivalent(Constraint c) {
		// TODO Auto-generated method stub
		return false;
	}

}
