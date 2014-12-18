package se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets;

import java.util.Vector;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintNetworkMarking;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.ValueOrderingH;
import org.metacsp.framework.Variable;
import org.metacsp.framework.VariableOrderingH;
import org.metacsp.framework.meta.MetaConstraint;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.framework.multi.MultiConstraintSolver;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.time.APSPSolver;
import org.metacsp.time.Bounds;
import org.sat4j.core.Vec;

import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;

public class SimpleMoveBasePlanner extends MetaConstraint {

	private static final long serialVersionUID = 1107282719383305518L;

	public SimpleMoveBasePlanner(VariableOrderingH varOH, ValueOrderingH valOH) {
		super(varOH, valOH);
	}

	@Override
	public ConstraintNetwork[] getMetaVariables() {
		Variable[] vars = this.getGroundSolver().getVariables();
		Vector<ConstraintNetwork> ret = new Vector<ConstraintNetwork>();
		for(Variable var : vars)
			if (var.getMarking().equals(LuciaMetaConstraintSolver.Markings.UNSUPPORTED)) {
				ConstraintNetwork cn = new ConstraintNetwork(null);
				cn.addVariable(var);
				ret.add(cn);
			}
		if (ret.isEmpty()) return null;
		return ret.toArray(new ConstraintNetwork[ret.size()]);
	}

	@Override
	public ConstraintNetwork[] getMetaValues(MetaVariable metaVariable) {
		ConstraintNetwork ret = new ConstraintNetwork(null);
		Variable goal = metaVariable.getConstraintNetwork().getVariables()[0];
		goal.setMarking(LuciaMetaConstraintSolver.Markings.SUPPORTED);
		//System.out.println("Achieving goal " + goal);
		//Create actions for the mini-plan
		Variable moveAction = this.getActivityNetworkSolver().createVariable(goal.getComponent());
		((SymbolicVariableActivity)moveAction).setSymbolicDomain("move_base");
		
		//Create constraints for the mini-plan
		moveAction.setMarking(LuciaMetaConstraintSolver.Markings.SUPPORTED);
		AllenIntervalConstraint durationMove = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Duration, new Bounds(3000,APSPSolver.INF));
		durationMove.setFrom(moveAction);
		durationMove.setTo(moveAction);
		AllenIntervalConstraint moveMeetsGoal = new AllenIntervalConstraint(AllenIntervalConstraint.Type.Meets);
		moveMeetsGoal.setFrom(moveAction);
		moveMeetsGoal.setTo(((SpatioTemporalSet)goal).getActivity());
		
		System.out.println("MoveMeetsGoal: " + moveMeetsGoal);
		
		ret.addConstraints(durationMove,moveMeetsGoal/*,previousObserveBeforePlan*/);
		ret.setAnnotation(this);
		
		return new ConstraintNetwork[] {ret};
	}

	@Override
	public void markResolvedSub(MetaVariable metaVariable,
			ConstraintNetwork metaValue) {
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
	
	public ConstraintSolver getActivityNetworkSolver() {
		return MultiConstraintSolver.getConstraintSolver(this.metaCS, ActivityNetworkSolver.class);
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
