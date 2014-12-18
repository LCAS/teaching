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
import org.metacsp.multi.activity.SymbolicVariableActivity;
import org.metacsp.multi.activity.ActivityNetworkSolver;
import org.metacsp.multi.symbols.SymbolicValueConstraint;
import org.metacsp.multi.symbols.SymbolicValueConstraint.Type;
import org.metacsp.multi.symbols.SymbolicVariableConstraintSolver;
import org.metacsp.utility.logging.MetaCSPLogging;

public class TestForLucia2014ThreeSmall {
	
	public static void main(String[] args) {
		MetaCSPLogging.setLevel(Level.FINEST);
		
		String[] symbols = new String[] {"A","B","C"};
		ActivityNetworkSolver solver = new ActivityNetworkSolver(0, 1000000, symbols);
		SymbolicVariableConstraintSolver groundSolver = ((SymbolicVariableConstraintSolver)solver.getConstraintSolvers()[1]);
		groundSolver.setSingleValue(false);
		
		SymbolicVariableActivity one = (SymbolicVariableActivity)solver.createVariable("one");
		SymbolicVariableActivity oneA = (SymbolicVariableActivity)solver.createVariable("oneA");
		one.setSymbolicDomain("A","C");
		
		SymbolicValueConstraint conDiff = new SymbolicValueConstraint(Type.DIFFERENT);
		conDiff.setFrom(one);
		conDiff.setTo(oneA);
		solver.addConstraint(conDiff);

		SymbolicVariableActivity two = (SymbolicVariableActivity)solver.createVariable("two");
		two.setSymbolicDomain("B","C");
		
		SymbolicValueConstraint conContains1 = new SymbolicValueConstraint(Type.CONTAINS);
		conContains1.setFrom(two);
		conContains1.setTo(one);		
		solver.addConstraint(conContains1);
		
		SymbolicValueConstraint conContains2 = new SymbolicValueConstraint(Type.CONTAINS);
		conContains2.setFrom(two);
		conContains2.setTo(oneA);	
		solver.addConstraint(conContains2);		
		
		ConstraintNetwork.draw(solver.getConstraintNetwork());
		
		//System.out.println("Added constraint? " + solver.addConstraints(conContains1,conContains2,conDiff));
		//System.out.println("Done");

	}

}
