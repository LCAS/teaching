/*******************************************************************************
 * Copyright (c) 2010-2013 Federico Pecora <federico.pecora@oru.se>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************/
package se.oru.aass.lucia_meta_csp_lecture;

import java.util.logging.Level;

import org.metacsp.framework.ConstraintNetwork;
import org.metacsp.framework.Variable;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint.Type;
import org.metacsp.multi.symbols.SymbolicVariable;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.utility.logging.MetaCSPLogging;

public class TestForLucia2014TwoBis {
	
	public static void main(String[] args) {
		MetaCSPLogging.setLevel(Level.FINEST);
		
		String[] symbols = new String[] {"B1","B2","B3","B4","B5"};
		SymbolicVariableConstraintSolver solver = new SymbolicVariableConstraintSolver(symbols, 100, true);
		solver.setSingleValue(false);
		
		int numRobots = 3;
		Variable[] vars = new Variable[numRobots];
		for (int i = 0; i < numRobots; i++) vars[i] = solver.createVariable("Robot"+i);
		
		ConstraintNetwork.draw(solver.getConstraintNetwork());
		
		((SymbolicVariable)vars[0]).setDomain(new String[] {"B1","B2","B3"});
//		SymbolicValueConstraint con01 = new SymbolicValueConstraint(Type.UNARYEQUALS);
//		con01.setUnaryValue(new boolean[] {true, true, false, false, false});
//		con01.setFrom(vars[0]);
//		con01.setTo(vars[0]);

		((SymbolicVariable)vars[1]).setDomain(new String[] {"B2","B3","B4"});
//		SymbolicValueConstraint con02 = new SymbolicValueConstraint(Type.UNARYEQUALS);
//		con02.setUnaryValue(new boolean[] {false, true, true, false, false});
//		con02.setFrom(vars[1]);
//		con02.setTo(vars[1]);
		
		((SymbolicVariable)vars[2]).setDomain(new String[] {"B3"});
//		SymbolicValueConstraint con03 = new SymbolicValueConstraint(Type.UNARYEQUALS);
//		con03.setUnaryValue(new boolean[] {false, false, true, false, false});
//		con03.setFrom(vars[2]);
//		con03.setTo(vars[2]);
		
		//alldifferent constraint
		SymbolicValueConstraint con = new SymbolicValueConstraint(Type.DIFFERENT);
		con.setScope(vars);
						
		try { Thread.sleep(1000); }
		catch (InterruptedException e) { e.printStackTrace(); }
		
		System.out.println("Added constraints? " + solver.addConstraint(con));
		
		try { Thread.sleep(1000); }
		catch (InterruptedException e) { e.printStackTrace(); }

		Variable varUnion1 = SymbolicVariableConstraintSolver.union(vars);
		System.out.println("Union of all vars (2): " + varUnion1);
		
		Variable varIntersection1 = SymbolicVariableConstraintSolver.intersection(vars);
		System.out.println("Intersection of all vars (2): " + varIntersection1);
	}

}
