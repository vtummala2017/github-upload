package edu.kit.aquaplanning;


import java.io.File;
import java.util.Date;

import java.io.FileWriter;
import java.io.IOException;
import java.net.InetAddress;
import java.util.List;
import java.io.File;

import edu.kit.aquaplanning.grounding.Grounder;
import edu.kit.aquaplanning.grounding.PlanningGraphGrounder;
import edu.kit.aquaplanning.grounding.htn.HtnGrounder;
import edu.kit.aquaplanning.model.ground.Action;
import edu.kit.aquaplanning.model.ground.Atom;
import edu.kit.aquaplanning.model.ground.GroundPlanningProblem;
import edu.kit.aquaplanning.model.ground.Plan;
import edu.kit.aquaplanning.model.ground.htn.HtnPlanningProblem;
import edu.kit.aquaplanning.model.lifted.PlanningProblem;
import edu.kit.aquaplanning.optimization.Clock;
import edu.kit.aquaplanning.optimization.SimplePlanOptimizer;
import edu.kit.aquaplanning.parsing.PlanParser;
import edu.kit.aquaplanning.parsing.ProblemParser;
import edu.kit.aquaplanning.planning.Planner;
import edu.kit.aquaplanning.planning.htn.TreeRexPlanner;
import edu.kit.aquaplanning.util.Logger;
import edu.kit.aquaplanning.validation.Validator;
import picocli.CommandLine;

/**
 * Reads a pair of PDDL files (domain and problem), does grounding,
 * and calls some planner in order to find a solution.
 */
public class Main2 {
	
	/**
	 * Parses the command line arguments and returns 
	 * a configuration object. Will print messages and exit 
	 * the application depending on requested help and/or
	 * errors which occur during parsing.
	 */
	public static Configuration parse(String[] args) {
		
		Configuration config = new Configuration();
		CommandLine cmd = new CommandLine(config);
		
		// Too few arguments? -> print usage, exit
		if (args.length < 2) {
			cmd.usage(System.out);
			System.exit(0);
		}
		
		// Parse arguments
		cmd.parse(args);
		
		// Usage and version information
		if (cmd.isUsageHelpRequested()) {
			cmd.usage(System.out);
			System.exit(0);
		}
		if (cmd.isVersionHelpRequested()) {
			cmd.printVersionHelp(System.out);
			System.exit(0);
		}
		
		return config;
	}
	
	/**
	 * Prints the provided plan to stdout. 
	 * If the config says so, also outputs the plan to a file.
	 */
	private static void printPlan(Configuration config, Plan plan) throws IOException {
		
		if (config.planOutputFile != null) {
			// Write plan to file
			FileWriter w = new FileWriter(config.planOutputFile);
			w.write(plan.toString());
			w.close();
			Logger.log(Logger.INFO, "Plan written to " + config.planOutputFile + ".");
		} else {
			// No output file => Always output plan to stdout
			Logger.log(Logger.ESSENTIAL, "Found plan:\n" + plan.toString());
		}
	}
	
	
	private static void mcall(String[] args) throws IOException {
		
		/*String filePath = args[1];
	      //Creating the File object
	      File file = new File(filePath);
	      //Getting the last modified time
	      long lastModified = file.lastModified();
	      Date date = new Date(lastModified);
	      //System.out.println("Given file was last modified at: ");
	      //System.out.println(date);
	      long lastModified2;// = file.lastModified();
	      Date date2;// = new Date(lastModified2);
		do 
		{
			date=null;	
			System.out.println("Round 1");
			System.in.read();*/
		// Read configuration from command line arguments
		Configuration config = parse(args);
		//System.out.println("Testing config");
		//System.out.println(config);
		Logger.init(config.verbosityLevel);
		
		// Welcome message
		Logger.log(Logger.INFO, "This is Aquaplanning - QUick Automated Planning.");
		Logger.log(Logger.INFO, "Running on " + InetAddress.getLocalHost().getHostName());
		
		// Configuration defaults are editable in Configuration.java.
		// For debugging, you can also override the configuration here, e.g.
		// config.heuristic = HeuristicType.manhattanGoalDistance;
		
		try {
			// Step 1: Parsing of domain and problem files
			Logger.log(Logger.INFO, "Parsing ...");
			PlanningProblem p = new ProblemParser().parse(config.domainFile, config.problemFile);
			//System.out.println(p.getInitialState());
			Logger.log(Logger.INFO_V, p.toString()); // print parsed problem
			Logger.log(Logger.INFO, "Parsing complete.\n");
			
			// Step 2: Grounding (to get "flat" sets of actions and atoms)
			Logger.log(Logger.INFO, "Grounding ...");
			Grounder grounder = new PlanningGraphGrounder(config);
			GroundPlanningProblem planningProblem = grounder.ground(p);
			System.out.println(planningProblem.getInitialState());
			if (planningProblem == null) {
				Logger.log(Logger.ESSENTIAL, "The problem has been found to be unsatisfiable. Exiting.");
				return;
			}
			// Print ground problem
			if (Logger.INFO_VV <= config.verbosityLevel) {				
				Logger.log(Logger.INFO_VV, planningProblem.toString());
			}
			Logger.log(Logger.INFO, "Grounding complete. " + planningProblem.getActions().size() 
					+ " actions resulted from the grounding.\n");
			Logger.log(Logger.INFO, "Ground problem contains " + (planningProblem.hasConditionalEffects() ? "some" : "no") + " conditional effects.");
			Logger.log(Logger.INFO, "Ground problem contains " + (planningProblem.hasComplexConditions() ? "some" : "no") + " complex conditions.");
			
			// Operation mode: Validation.
			if (config.planFileToValidate != null) {	
				// Validate a provided plan file
				Logger.log(Logger.INFO, "Parsing plan " + config.planFileToValidate + " ...");
				Plan plan = PlanParser.parsePlan(config.planFileToValidate, planningProblem);
				boolean isValid = false;
				if (plan != null) {
					Logger.log(Logger.INFO, "Validating plan ...");
					isValid = Validator.planIsValid(planningProblem, plan);
				}
				Logger.log(Logger.ESSENTIAL, "PLAN " + (isValid ? "VALID" : "INVALID") + ". Exiting.");					
				return;
			}
			
			// Operation mode: Planning.
			Plan plan = null;
			if (p instanceof HtnPlanningProblem) {
				// HTN planning problem
				
				Logger.log(Logger.INFO, "Initializing HTN grounding ...");
				HtnGrounder htnGrounder = new HtnGrounder(planningProblem, (PlanningGraphGrounder) grounder);
				TreeRexPlanner planner = new TreeRexPlanner(config, htnGrounder);
				plan = planner.findPlan();
				
			} else {
				// Classical planning problem
				
				// Step 3: Planning
				Logger.log(Logger.INFO, "Planning ...");
				Planner planner = Planner.getPlanner(config);
				//Atom atom2 = new Atom(60, "(at-block block5 rooma)",true);
				//planningProblem.get
				//.add(atom2);
				//List<Action> actions = planningProblem.getActions();
				//actions.add(e);
				plan = planner.findPlan(planningProblem);
				//String s = findPlanTry(planningProblem); 
			}			
		
		
			// Solution found?
			if (plan == null) {
				// -- no
				
				Logger.log(Logger.ESSENTIAL, "Planner did not find any solution.");
				
			} else {
				// -- yes
				
				Logger.log(Logger.INFO, "Planner finished with a plan of length " 
						+ plan.getLength() + ".");
				
				if (config.optimizePlan) {
					// Employ plan optimization
					
					Logger.log(Logger.INFO, "Plan optimization ...");
					SimplePlanOptimizer o = new SimplePlanOptimizer(planningProblem);
					plan = o.improvePlan(plan, new Clock(5000)); // TODO set proper time limit
					Logger.log(Logger.INFO, "Final plan has a length of " + plan.getLength() + ".");
					printPlan(config, plan);
				}
				
				// Step 4: Validate plan (directly outputting any errors)
				Logger.log(Logger.INFO, "Validating ...");
				if (Validator.planIsValid(planningProblem, plan)) {
					Logger.log(Logger.INFO, "Plan has been found to be valid.");
					printPlan(config, plan);
				}
			}
			
			
		} catch (Exception e) {
			Logger.log(Logger.ERROR, "An internal error occurred.");
			e.printStackTrace();
		}
		//lastModified2 = file.lastModified();
	    //date2 = new Date(lastModified2);
		//}while(date2.compareTo(date)>0);
		Logger.log(Logger.INFO, "Aquaplanning exiting.");
		
	}
		
	public static void main(String[] args) throws Exception {
		args[0]="/Users/vijayanthtummala/Downloads/trydomain.pddl";
		args[1]="/Users/vijayanthtummala/Downloads/aquaplanning-master/filename0.pddl";
		
		String path="/Users/vijayanthtummala/Downloads/aquaplanning-master/";
		String fileprefix = "filename";
		String fileType =".pddl";
		
		mcall(args);
		
		int i=0;
		boolean hasNext = true;
		
		
		
		while (hasNext)
		{
			String filePath = path+fileprefix+String.valueOf(i)+fileType;
			args[0]="/Users/vijayanthtummala/Downloads/trydomain.pddl";
			args[1]=filePath;
			
			File f = new File(filePath);
			if(f.exists() && !f.isDirectory()) { 
				mcall(args);
			}
			else {
				hasNext=false;
			}
			
			
			mcall(args);
		}
		/* File dir = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/MyStateFiles");
		  File[] directoryListing = dir.listFiles();
		 //int counter=21;
		 while (directoryListing != null) {
			  //while (counter!= 0) {
		    for (File child : directoryListing) {
		    	String tc = child.toString();
		    	if (tc.contains(".pddl"))
		    	{	
		    		System.out.println(child);
		    		args[1]= tc;
		    		mcall(args);
		    	}
		    }
		    //counter--;
		  }*/
	}
}
