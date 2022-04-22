package edu.kit.aquaplanning.planning.sat;

import java.util.List;

import edu.kit.aquaplanning.Configuration;
import edu.kit.aquaplanning.model.ground.GroundPlanningProblem;
import edu.kit.aquaplanning.model.ground.Plan;
import edu.kit.aquaplanning.planning.Planner;
import edu.kit.aquaplanning.sat.Sat4jSolver;
import edu.kit.aquaplanning.sat.SymbolicReachabilityFormula;
import edu.kit.aquaplanning.sat.SymbolicReachabilitySolver;
import edu.kit.aquaplanning.sat.encoders.ForeachEncoding;
import edu.kit.aquaplanning.sat.encoders.PlanningToSatEncoder;

public class SymbolicReachabilityPlanner extends Planner {

	public SymbolicReachabilityPlanner(Configuration config) {
		super(config);
	}

	@Override
	public Plan findPlan(GroundPlanningProblem problem) {
		PlanningToSatEncoder encoder = new ForeachEncoding();
		SymbolicReachabilityFormula fla = encoder.encodeProblem(problem);
		SymbolicReachabilitySolver solver = new SymbolicReachabilitySolver(new Sat4jSolver());
		List<int[]> model = solver.solve(fla);
		if (model != null) {
			return encoder.decodePlan(problem, model);
		} else {
			return null; 
		}
	}

	@Override
	public Plan findPlanAgain(GroundPlanningProblem planningProblem) {
		// TODO Auto-generated method stub
		return null;
	}

}
