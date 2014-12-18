package se.oru.aass.lucia_meta_csp_lecture.meta.spaceTimeSets;

import org.metacsp.framework.Constraint;
import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.ConstraintSolver;
import org.metacsp.framework.Variable;
import org.metacsp.framework.meta.MetaConstraintSolver;
import org.metacsp.framework.meta.MetaVariable;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.allenInterval.AllenIntervalConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.spatial.geometry.GeometricConstraint;

import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSet;
import se.oru.aass.lucia_meta_csp_lecture.multi.spaceTimeSets.SpatioTemporalSetNetworkSolver;

public class LuciaMetaConstraintSolver extends MetaConstraintSolver {

	private static final long serialVersionUID = 7625260983386681625L;
	
	private SymbolicVariableActivity future = null; 
	
	public static enum Markings {SUPPORTED, UNSUPPORTED};

	protected LuciaMetaConstraintSolver(Class<?>[] constraintTypes, long animationTime, ConstraintSolver[] internalSolvers) {
		super(constraintTypes, animationTime, internalSolvers);
	}
	
	public boolean isSensorReading(SpatioTemporalSet var) {
		SpatioTemporalSetNetworkSolver spatioTemporalSetSolver = (SpatioTemporalSetNetworkSolver)this.getConstraintSolvers()[0];
		ActivityNetworkSolver activitySolver = spatioTemporalSetSolver.getActivitySolver();
		if (future == null) {
			Variable[] acts = activitySolver.getVariables("Time");
			for (Variable act : acts) {
				if (((SymbolicVariableActivity)act).getSymbolicVariable().getSymbols()[0].equals("Future")) {
					future = (SymbolicVariableActivity)act;
					break;
				}
			}
		}
		SpatioTemporalSet act = ((SpatioTemporalSet)var);
		Constraint[] cons = activitySolver.getConstraintNetwork().getConstraints(act.getActivity(), future);
		if (cons != null && cons.length != 0) return true;
		return false;
	}
	
	public LuciaMetaConstraintSolver(long origin, long horizon, int numActivities, String[] symbols) {
		super(new Class[] {AllenIntervalConstraint.class, SymbolicValueConstraint.class, GeometricConstraint.class}, 0, new SpatioTemporalSetNetworkSolver(origin, horizon, numActivities, symbols));
	}
	
	@Override
	public void preBacktrack() {
		// TODO Auto-generated method stub
	}

	@Override
	public void postBacktrack(MetaVariable metaVariable) {
		// TODO Auto-generated method stub
	}

	@Override
	protected void retractResolverSub(ConstraintNetwork metaVariable,
			ConstraintNetwork metaValue) {
		// TODO Auto-generated method stub
	}

	@Override
	protected boolean addResolverSub(ConstraintNetwork metaVariable,
			ConstraintNetwork metaValue) {
		// TODO Auto-generated method stub
		return true;
	}

	@Override
	protected double getUpperBound() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected void setUpperBound() {
		// TODO Auto-generated method stub
	}

	@Override
	protected double getLowerBound() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected void setLowerBound() {
		// TODO Auto-generated method stub
	}

	@Override
	protected boolean hasConflictClause(ConstraintNetwork metaValue) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void resetFalseClause() {
		// TODO Auto-generated method stub
	}

}
