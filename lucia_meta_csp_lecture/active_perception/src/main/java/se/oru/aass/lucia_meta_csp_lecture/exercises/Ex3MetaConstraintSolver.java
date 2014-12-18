package se.oru.aass.lucia_meta_csp_lecture.exercises;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.meta.MetaConstraintSolver;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint;




public class Ex3MetaConstraintSolver extends MetaConstraintSolver{

	protected Ex3MetaConstraintSolver(Class<?>[] constraintTypes,
			long animationTime, ConstraintSolver[] internalSolvers) {
		super(constraintTypes, animationTime, internalSolvers);
	}

	public Ex3MetaConstraintSolver(long origin, long horizon, int numActivities) {
		super(new Class[] {AllenIntervalConstraint.class, SymbolicValueConstraint.class}, 0, new ActivityNetworkSolver(origin, horizon, 500));
	}
	
	@Override
	protected boolean addResolverSub(ConstraintNetwork arg0,
			ConstraintNetwork arg1) {
		// TODO Auto-generated method stub
		return true;
	}

	@Override
	protected double getLowerBound() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double getUpperBound() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected boolean hasConflictClause(ConstraintNetwork arg0) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void postBacktrack(MetaVariable arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void preBacktrack() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void resetFalseClause() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void retractResolverSub(ConstraintNetwork arg0,
			ConstraintNetwork arg1) {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void setLowerBound() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void setUpperBound() {
		// TODO Auto-generated method stub
		
	}

}

