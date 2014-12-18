package se.oru.aass.lucia_meta_csp_lecture;

import java.util.Vector;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.spatial.geometry.GeometricConstraint;
import org.metacsp.spatial.geometry.GeometricConstraintSolver;
import org.metacsp.spatial.geometry.Polygon;
import org.metacsp.spatial.geometry.Vec2;
import org.metacsp.utility.UI.PolygonFrame;


public class TestGeometricConstraintSolverSmall {

	public static void main(String[] args) {

		GeometricConstraintSolver solver = new GeometricConstraintSolver();
		Variable var = solver.createVariable();
		
		Polygon p0 = (Polygon)var;
		Vector<Vec2> vecs1 = new Vector<Vec2>();
		vecs1.add(new Vec2(100,87));
		vecs1.add(new Vec2(60,30));
		vecs1.add(new Vec2(220,60));
		vecs1.add(new Vec2(180,120));
		p0.setDomain(vecs1.toArray(new Vec2[vecs1.size()]));
		p0.setMovable(true);
		
		ConstraintNetwork.draw(solver.getConstraintNetwork());
		PolygonFrame pf = new PolygonFrame("Polygon Constraint Network", solver.getConstraintNetwork());
	}

}
