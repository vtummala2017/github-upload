package edu.kit.aquaplanning.planning;

import java.io.File;  // Import the File class
import java.io.FileReader;
import java.io.IOException;  // Import the IOException class to handle errors
import java.net.InetAddress;
import java.io.FileWriter;   // Import the FileWriter class


import java.util.*;

import org.apache.commons.lang3.ArrayUtils;
import org.apache.commons.lang3.StringUtils;

import edu.kit.aquaplanning.Configuration;
import edu.kit.aquaplanning.Main;
import edu.kit.aquaplanning.grounding.Grounder;
import edu.kit.aquaplanning.grounding.PlanningGraphGrounder;
import edu.kit.aquaplanning.grounding.htn.HtnGrounder;
import edu.kit.aquaplanning.model.ground.Action;
import edu.kit.aquaplanning.model.ground.Atom;
import edu.kit.aquaplanning.model.ground.AtomSet;
import edu.kit.aquaplanning.model.ground.Goal;
import edu.kit.aquaplanning.model.ground.GroundPlanningProblem;
import edu.kit.aquaplanning.model.ground.Plan;
import edu.kit.aquaplanning.model.ground.State;
import edu.kit.aquaplanning.model.ground.htn.HtnPlanningProblem;
import edu.kit.aquaplanning.model.lifted.PlanningProblem;
import edu.kit.aquaplanning.optimization.Clock;
import edu.kit.aquaplanning.optimization.SimplePlanOptimizer;
import edu.kit.aquaplanning.parsing.PlanParser;
import edu.kit.aquaplanning.parsing.ProblemParser;
import edu.kit.aquaplanning.planning.datastructures.ActionIndex;
import edu.kit.aquaplanning.planning.datastructures.SearchNode;
import edu.kit.aquaplanning.planning.datastructures.SearchQueue;
import edu.kit.aquaplanning.planning.datastructures.SearchStrategy;
import edu.kit.aquaplanning.planning.heuristic.Heuristic;
import edu.kit.aquaplanning.planning.htn.TreeRexPlanner;
import edu.kit.aquaplanning.util.Logger;
import edu.kit.aquaplanning.validation.Validator;
import picocli.CommandLine;

/**
 * Generic state space forward search planner. Different search strategies 
 * and heuristics may be employed.
 * 
 * Based on Algorithm 2.2 from "Automated Planning and Acting" 
 * by Malik Ghallab et al., 2016.
 * 
 * @author Dominik Schreiber
 */
public class ForwardSearchPlanner extends Planner {
	
	public ForwardSearchPlanner(Configuration config) {
		super(config);
	}
	

	private static boolean isStringUpperCase(String str){
        
        //convert String to char array
        char[] charArray = str.toCharArray();
        
        for(int i=0; i < charArray.length; i++){
            
            //if any character is not in upper case, return false
            if( !Character.isUpperCase( charArray[i] ))
                return false;
        }
        
        return true;
    }
	
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
	
	public static void maincopy(String[] args)
	{
		Configuration config = parse(args);
		//System.out.println("Testing config");
		//System.out.println(config);
		Logger.init(config.verbosityLevel);
		
		// Welcome message
		Logger.log(Logger.INFO, "This is Aquaplanning - QUick Automated Planning.");
		//Logger.log(Logger.INFO, "Running on " + InetAddress.getLocalHost().getHostName());
		
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
	
	
	/**
	 * Given a ground planning problem, employs a forward 
	 * state space search procedure according to the configuration
	 * which was provided to this object's constructor.
	 */
	@Override
	public Plan findPlan(GroundPlanningProblem problem) {
		startSearch();
		//Start of My Code
		boolean sendData= false;
		
		// create token1
	    String gtoken1 = "";
	    List<String> gtt = new ArrayList<String>();
		
		// create token1
	    String token1 = "";
	    List<String> tt = new ArrayList<String>();
	   
	    
	    
	 // create token1
	    String token2 = "";
	    List<String> temps2 = new ArrayList<String>();
	    
	 // create token1
	    String robottoken1 = "";
	    List<String> rtt = new ArrayList<String>();
	   
	    
	 // create token1
	    String robottoken11 = "";
	    List<String> rtt1 = new ArrayList<String>();
	    
	    
	    String finalRoomSend = "";
	    
	    String state_domain ="";
	    String state_object = "";
	    String state_goal ="";
	    String state_init="";
	    int [] tall = new int [100];
	    try
	    {
	    	
	    	// create Scanner inFile1
		    Scanner gt = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/GoalAndTypes.txt")).useDelimiter("[;\\r\\n]");

		    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
		  
		   

		    // while loop
		    while (gt.hasNext()) {
		      // find next line
		      gtoken1 = gt.next();
		      gtt.add(gtoken1);
		    }
		    //System.out.println(tt.size());
		    //System.out.println(tt.get(0));
		    //System.out.println(tt.get(1));
		    //System.out.println(tt.get(2));
		    //System.out.println(tt.get(3));
		    gt.close();
		    System.out.println(gtt);
		    
		
		
	    // create Scanner inFile1
	    Scanner inFile1 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/iplist.txt")).useDelimiter("[;\\r\\n]");

	    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
	  
	   

	    // while loop
	    while (inFile1.hasNext()) {
	      // find next line
	      token1 = inFile1.next();
	      tt.add(token1);
	    }
	    //System.out.println(tt.size());
	    //System.out.println(tt.get(0));
	    //System.out.println(tt.get(1));
	    //System.out.println(tt.get(2));
	    //System.out.println(tt.get(3));
	    inFile1.close();
	    
	    // create Scanner inFile1
	    Scanner inFile11 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/robotselfdata.txt")).useDelimiter("[;\\r\\n]");

	    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
	  
	   
	    int rcount=0;
	    // while loop
	    while (inFile11.hasNext()) {
	      // find next line
	    	robottoken1 = inFile11.next();
	    	if(rcount==1) 
	    	{
	    		 rtt.add(robottoken1);
	    	}
	      rcount=1;
	     
	    }
	    //System.out.println(tt.size());
	    //System.out.println(tt.get(0));
	    //System.out.println(tt.get(1));
	    //System.out.println(tt.get(2));
	    //System.out.println(tt.get(3));
	    inFile11.close();
	    
	    // create Scanner inFile1
	    Scanner inFile111 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/robotselfdata.txt")).useDelimiter("[;\\r\\n]");

	    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
	  
	   
	    int rcount1=0;
	    // while loop
	    while (inFile111.hasNext()) {
	      // find next line
	    	robottoken11 = inFile111.next();
	    	if(rcount1==1) 
	    	{
	    		 rtt1.add(robottoken1);
	    	}
	      rcount1=1;
	     
	    }
	    //System.out.println(tt.size());
	    //System.out.println(tt.get(0));
	    //System.out.println(tt.get(1));
	    //System.out.println(tt.get(2));
	    //System.out.println(tt.get(3));
	    inFile111.close();
	    
	 // for-each loop for calculating heat index of May - October

	    // create Scanner inFile1
	    Scanner inFile2 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/IdentifiedRoomList.txt")).useDelimiter("[;\n]");

	    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
	    // List<String> temps = new LinkedList<String>();
	    

	    // while loop
	    while (inFile2.hasNext()) {
	      // find next line
	      token2 = inFile2.next();
	      temps2.add(token2);
	    }
	    inFile2.close();
	    
	    }
	    catch(Exception e)
	    {
	    	
	    }
		
	    
	    
		ArrayList<String> SMMactions=new ArrayList<String>(); 
		ArrayList<String> SMMStates=new ArrayList<String>();
		ArrayList<String> ourplan=new ArrayList<String>();
		 //String[] IP = {"move ", "move2 ", "pickup ", "drop "};
	     //   int size = IP.length;
	        //for (int i =0;i<size;i++)
	        //System.out.println(IP[i]);
		//End of My Code
		/*System.out.println(problem.getGoal());
		
		Atom atomv1 = new Atom(60, "(visited rooma)",true);
		Atom atomv2 = new Atom(61, "(visited roomb)",true);
		Atom atomv3 = new Atom(62, "(visited roomc)",true);
		Atom atomv4 = new Atom(63, "(visited roomd)",true);
		Atom atomv5 = new Atom(64, "(visited roome)",true);
		Atom atomv6 = new Atom(64, "(visited roomf)",true);
		Atom atom1 = new Atom(70, "(at-block block1 rooma)",true);
		Atom atom2 = new Atom(71, "(at-block block2 rooma)",true);
		Atom atom3 = new Atom(72, "(at-block block3 rooma)",true);
		Atom atom4 = new Atom(73, "(at-block block4 rooma)",true);
		Atom atom5 = new Atom(74, "(at-block block5 rooma)",true);
		List<Atom> initialStateAtoms = new ArrayList<>();
		initialStateAtoms.add(atomv1);
		initialStateAtoms.add(atomv2);
		initialStateAtoms.add(atomv3);
		initialStateAtoms.add(atomv4);
		initialStateAtoms.add(atomv5);
		initialStateAtoms.add(atomv6);
		initialStateAtoms.add(atom1);
		initialStateAtoms.add(atom2);
		initialStateAtoms.add(atom3);
		initialStateAtoms.add(atom4);
		//initialStateAtoms.add(atom5);
		Goal g = new Goal(initialStateAtoms);
		problem.setGoal(g);
		System.out.println(problem.getGoal());*/
		Logger.log(Logger.INFO, "Starting forward search with " + config.toString());
		// Important objects from the planning problem
		
		//Atom atom2 = new Atom(84, "(at-block block5 rooma)",true);
		//List<Atom> initialStateAtoms = new ArrayList<>();
		//initialStateAtoms.add(atom2);
		//System.out.println(initialStateAtoms);
		//AtomSet am = new AtomSet(initialStateAtoms);
		//State st = new State(initialStateAtoms);
		//st.addAll(am);
		//atomsl.set(atom2);
		//System.out.println(atom.getId());
		//System.out.println();
		//State st = problem.getInitialState();
		//System.out.println(problem.stateToString(st));
		//problem.setInitialState(s);
		//st.set(atom2);
		//System.out.println(problem.stateToString(st));
		//problem.setInitialState(st);
		//System.out.println(problem.stateToString(st));
		
		State initState = problem.getInitialState();
		System.out.println(initState);
		
		//initState.set(atom);
		//System.out.println("hereee");
		System.out.println(problem.stateToString(initState));
		System.out.println(problem.getInitialState().toString());
		//System.out.println(initState);
		Goal goal = problem.getGoal();
		System.out.println(goal);
		ActionIndex aindex = new ActionIndex(problem);
		System.out.println(aindex);
		// Initialize forward search
		SearchQueue frontier;
		SearchStrategy strategy = new SearchStrategy(config);
		if (strategy.isHeuristical()) {
			Heuristic heuristic = Heuristic.getHeuristic(problem, config);
			frontier = new SearchQueue(strategy, heuristic);
		} else {
			frontier = new SearchQueue(strategy);
		}
		frontier.add(new SearchNode(null, initState));
		
		int iteration = 1;
		int visitedNodesPrintInterval = 28;
		long timeStart = System.nanoTime();
		
		//System.out.println(problem.getAtomNames());
		int shareddatausedcount=0; 
		while (withinComputationalBounds(iteration) && !frontier.isEmpty()) {
			
			// Visit node (by the heuristic provided to the priority queue)
			SearchNode node = frontier.get();
			
			// Is the goal reached?
			if (goal.isSatisfied(node.state)) {
				
				// Extract plan
				Plan plan = new Plan();
				
				
				while (node != null && node.lastAction != null) {
					plan.appendAtFront(node.lastAction);
					//ourplan.add(node.lastAction.getName());
					//SMMStates.add(problem.stateToString(node.state));
					
					if (node.lastAction.getName().contains("robot1"))
					{
						ourplan.add(node.lastAction.getName());
						SMMStates.add(problem.stateToString(node.state));
						
					}
					//System.out.println(node.lastAction.getName());
					//System.out.println(problem.stateToString(node.state));
					
					/*String state_domain = "(define (problem move_to_b) (:domain BW4TT) \n";
					String state_object = "(:objects \n" + 
							"        rooma roomb roomc roomd roome roomf - room\n" + 
							"        block1 block2 block3 block4 - block\n" + 
							"        robot1 robot2 - robot\n" + 
							"    ) \n";
					String state_goal = " (:goal\n" + 
							"        (and\n" + 
							"          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))\n" + 
							"        )\n" + 
							"    )\n" + 
							")";
					String state_init = problem.stateToString(node.state);
					state_init = state_init.replace("(_TRUE)", "");
					state_init = state_init.replace("{", "");
					state_init = state_init.replace("}", "");
					state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) \n" + 
							"        (BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n" + 
							"        (ROBOT robot1) (ROBOT robot2) \n" + state_init+ "\n ) \n";
					try {
					      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/filename.pddl");
					      myWriter.write( state_domain + state_object + state_init + state_goal);
					      myWriter.close();
					      System.out.println("Successfully wrote to the file.");
					    } catch (IOException e) {
					      System.out.println("An error occurred.");
					      e.printStackTrace();
					    }
					try {
					      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/finalstepitaration.txt");
					      myWriter.write( node.lastAction.getName() + " \n" + problem.stateToString(node.state) );
					      myWriter.close();
					      System.out.println("Successfully wrote to the file.");
					    } catch (IOException e) {
					      System.out.println("An error occurred.");
					      e.printStackTrace();
					    }
					//System.out.println("Lets c");
					//System.exit(0);*/
					
					
					//System.out.println(node.state);
					//node.state =problem.stateToString(node.state);
					//problem.setInitialState(node.state);
					node = node.parent;
				}
				
				Collections.reverse(ourplan);
				Collections.reverse(SMMactions);
				Collections.reverse(SMMStates);
				//System.out.println(SMMactions);
				//System.out.println(SMMStates);
				//System.out.println(ourplan);
				System.out.println("ourplan");
				//System.out.println(ourplan.size());
				
				
				int [] tll = new int [2];
				FileWriter myCW;
				try {
					//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
					//System.out.println(reader.read());
					
					Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/check.txt"));
					
					int t = 0;
					while(sc.hasNextInt())
					{
					     tll[t++] = sc.nextInt();
					}
					System.out.println(tll[0]);
					/*if (tll[0] == 0)
					{
						tll[0] =1;
						System.out.println(tll[0]);
					}
					else
					{
						
						tll[0] = 1;
					}*/
					
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				
				int shareddatausedcountrl = 0;
				
				try {
					//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
					//System.out.println(reader.read());
					
					Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/shareddatausedcountrobotrelated.txt"));
					
					int t = 0;
					while(sc.hasNextInt())
					{
						shareddatausedcountrl  = sc.nextInt();
					}
					//System.out.println(tll[0]);
					
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				
				
				int counter =0;
				
				try
				{
				
				
					
					
					// create token1
			    String tok = "";

			    // for-each loop

			    // create Scanner inFile1
			    //Scanner inFile1 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblemnew.pddl")).useDelimiter(",\\s*");
			    Scanner inFile1 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/tryproblemnewtestr1.pddl")).useDelimiter(",\\s*");
			    
			    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
			    // List<String> temps = new LinkedList<String>();
			    List<String> temps = new ArrayList<String>();

			    // while loop
			    while (inFile1.hasNext()) {
			      // find next line
			      tok += inFile1.next();
			      temps.add(tok);
			    }
			    inFile1.close();

			    //System.out.println(token1);
			    
			    
			    //String s="test string(67)";
			    //String requiredString = token1.substring(token1.indexOf("(:") + 1, token1.indexOf("\n)"));
			    //System.out.println(requiredString);
			    
			    
			    String[] envVars =StringUtils.substringsBetween(token1, "(:object", ")");
			    String[] envVars2 =StringUtils.substringsBetween(token1, "(:init", ")\n)");
			    String[] envVars3 =StringUtils.substringsBetween(token1, "(:goal", "))\\n)");
			    //System.out.println(test[0]);
			    
			    if (null != envVars) {
			        for (int i = 0; i < envVars.length; i++) {
			          String envKey = envVars[i];
			          //String envVal = System.getenv(envKey);
			          System.out.println(envKey);
			          //ret = StringUtils.replace(ret, "${" + envKey + "}", envVal);
			        }
			    
				}
			    
			    if (null != envVars2) {
			        for (int i = 0; i < envVars2.length; i++) {
			          String envKey = envVars2[i];
			          //String envVal = System.getenv(envKey);
			          System.out.println(envKey);
			          //ret = StringUtils.replace(ret, "${" + envKey + "}", envVal);
			        }
			    
				}
			    System.out.println(3);
			    if (null != envVars3) {
			        for (int i = 0; i < envVars3.length; i++) {
			        	
			          String envKey = envVars3[i];
			          //String envVal = System.getenv(envKey);
			          System.out.println(envKey);
			          //ret = StringUtils.replace(ret, "${" + envKey + "}", envVal);
			        }
			    
				}
				}
				catch(Exception e)
				{
					
					
				}
				
				
				if(ourplan.size()>0 && tll[0]==0) {
					
					try {
						tll[0]=1;
						myCW = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/check.txt");
						myCW.write(new Integer(tll[0]).toString());
					    myCW.close();
					} catch (IOException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
				
					
				
				    
					boolean addcheck=false;  
				    
				
					outerloop2: 
				//for (int i = 0; i < ourplan.size(); i ++) {
					for (int i = 0; i < ourplan.size(); i ++) {
					
						if(i==1)
						{
						    return plan;
						}
						
					/*	
						try {
			Thread.sleep(50);
		} catch (InterruptedException e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}
			*/			
					System.out.println("IF Part");
					System.out.println(ourplan.get(i));
					System.out.println(SMMStates.get(i));
					
					
					System.out.println(tt.size());
					for (int j = 0; j<tt.size();j++)
					{
						//tt.get(j);
						if (ourplan.get(i).contains(tt.get(j)))
						{
							//System.out.println("Vijay");
							//System.out.println(j);
							 
							for (int k = 0; k<temps2.size();k++)
							{
								if (ourplan.get(i).contains(temps2.get(k)))
								{
									
									//System.out.println("Vijay2");
									//System.out.println(j);
									System.out.println("Start of Communication");
									System.out.println(ourplan.get(i));
									System.out.println(SMMStates.get(i));
									System.out.println("End of Communication");
									sendData=true;
									finalRoomSend = temps2.get(k);
								}
							}
							
						}
					}
					
					String OG = "";
					String SG = "";
					List<String> ts = new ArrayList<String>();
					List<String> ts2 = new ArrayList<String>();
					List<String> ts2temp = new ArrayList<String>();
					String blocks;
					String addstateobj="";
					boolean changeobj= false;
					try {
						
					
						

				    // for-each loop

				    // create Scanner inFile1
				    //Scanner if1 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/ObjectSent.txt")).useDelimiter("\n");
						Scanner if1 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/ObjectSent.txt"));
				    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
				    // List<String> temps = new LinkedList<String>();
				    

				    // while loop
				    while (if1.hasNext()) {
				      // find next line
				    	String tp; 
				    	tp = if1.next();
				      OG += tp;
				      ts.add(tp);
				    }
				    //Scanner if2 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/StatePredicatesSent.txt")).useDelimiter("\n");
				    Scanner if2 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/StatePredicatesSent.txt")).useDelimiter(",\\s*");

					    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
					    // List<String> temps = new LinkedList<String>();
					    

					    // while loop
					    while (if2.hasNext()) {
					      // find next line
					    	String tp; 
					    	tp = if2.next();
					      SG += tp;
					      //SG += if2.next();
					      ts2.add(tp);
					    }
				    if2.close();
				    	}
					catch(Exception e)
					{
						
					}
					
					try {
					      File myObj = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/StatePredicatesSent.txt");
					      Scanner myReader = new Scanner(myObj).useDelimiter(",\\s*");;
					      while (myReader.hasNextLine()) {
					        String data = myReader.nextLine();
					        System.out.println(data);
					        SG += data;
						      //SG += if2.next();
						      ts2.add(data);
					      }
					      myReader.close();
					    } catch (Exception e) {
					      //System.out.println("An error occurred.");
					      //e.printStackTrace();
					    }
					    
					int bdata=0;
					for (int i1 =0; i1<ts2.size();i1++)
					{
						if (ts2.get(i1).contains("()"))
						{
							bdata=1;
						}
						if(!(ts2.get(i1).contains("(")))
						{
							bdata=1;
						}
						if(ts2.get(i1).contains(rtt.get(0)))
						{
							bdata=1;
						}
						if(bdata==0)
						{
							ts2temp.add(ts2.get(i1));
						}
						/*if(!(ts2.get(i1).contains(")")))
						{
							ts2.remove(i1);
						}*/
						bdata=0;
					}
					ts2.clear();
					
					for (int i1 =0; i1<ts2temp.size();i1++)
					{
						ts2.add(ts2temp.get(i1));
						
					}
					/*
					File myObj = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/ObjectSent.txt"); 
					    if (myObj.delete()) { 
					      System.out.println("Deleted the file: " + myObj.getName());
					    } else {
					      System.out.println("Failed to delete the file.");
					    } 
						
					    File myObj1 = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/StatePredicatesSent.txt"); 
					    if (myObj1.delete()) { 
					      System.out.println("Deleted the file: " + myObj1.getName());
					    } else {
					      System.out.println("Failed to delete the file.");
					    } 
					  */ 
					blocks = "block1 block2 block3 block4 - block\n";
					//blocks = "block1 block2 - block\n";
					int calculate =0; ///can be used to check how many times an input was accepted
					for (int e=0; e< ts.size(); e++)
					{
						if (!(blocks.contains(ts.get(e))))
						{
							blocks = ts.get(e) +" " +blocks;
							changeobj=true;
							calculate++;
						}
					}
					
					state_init = SMMStates.get(i);
					state_init = state_init.replace("(_TRUE)", "");
					state_init = state_init.replace("{", "");
					state_init = state_init.replace("}", "");
					state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  \n" + 
							"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
							//"(BLOCK block1) (BLOCK block2) ; (BLOCK block3) \n "
							+ " (at-block block1 roomc) (at-block block2 roomf) (at-block block3 roomj) (at-block block4 roome)" + 
							//+ "(at-block block1 rooma) (at-block block2 rooma)" + 	
							"(ROBOT robot1)\n"
							//"(ROBOT robot1) \n"
							+ "(free robot1) (visited rooma) (observing rooma robot1)  \n"
							+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
							//+ "(color-block block1 blue) (color-block block2 blue) \n"
							//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
							+ "   \n"+  "  \n" + state_init + addstateobj+ "\n ) \n";
							//+ "  \n"+  " \n" + state_init + "\n ) \n";
					System.out.println(state_init);
					
					for (int e=0; e< ts2.size(); e++)
					{
						if (!(state_init.contains(ts2.get(e))) && (ts2.get(e) != " ()") && (!(addstateobj.contains(ts2.get(e)))) && (ts2.get(e) != "()")  && (ts2.get(e) != "() "))
						{
							addstateobj = ts2.get(e) +" " +addstateobj;
							changeobj=true;
							calculate++;
						}
					}
					
					
					//roomfcounter=0;
					try {
						//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
						//System.out.println(reader.read());
						
						Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/shareddatausedcount.txt"));
						
						int t = 0;
						while(sc.hasNextInt())
						{
							shareddatausedcount  = sc.nextInt();
						}
						//System.out.println(tll[0]);
						
					} catch (IOException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					
					
					if (changeobj)
					{
						if (addcheck==false)
						{
						shareddatausedcount++;
						}
						
						state_init = SMMStates.get(i);
						state_init = state_init.replace("(_TRUE)", "");
						state_init = state_init.replace("{", "");
						state_init = state_init.replace("}", "");
						state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  \n" + 
								"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
								//"(BLOCK block1) (BLOCK block2) \n "
								+ " (at-block block1 roomc) (at-block block2 roomf) (at-block block3 roomj) (at-block block4 roome)" + 
								//+ "(at-block block1 rooma) (at-block block2 rooma)" + 	
								"(ROBOT robot1) \n"
								//"(ROBOT robot1) \n"
								+ "(free robot1) (visited rooma) (observing rooma robot1) \n"
								+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
								//+ "(color-block block1 blue) (color-block block2 blue) \n"
								//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
								// "  \n"+  " \n" + state_init + "\n ) \n";
								+ "  \n"+  " \n" + state_init + addstateobj+ "\n ) \n";
						System.out.println(state_init);
						/*
						state_init = SMMStates.get(i);
						state_init = state_init.replace("(_TRUE)", "");
						state_init = state_init.replace("{", "");
						state_init = state_init.replace("}", "");
						state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) \n" + 
								"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
								+ "(at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma)" + 
								"(ROBOT robot1) (ROBOT robot2) \n"
								+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
								//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
								+ "  (color-preference robot2 red) \n"+  " \n" + state_init+ "\n " + addstateobj + "\n ) \n";
						System.out.println(state_init);*/
					}
					
					try {
						//tll[0]=0;
						myCW = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/shareddatausedcount.txt");
						myCW.write(new Integer(shareddatausedcount).toString());
					    myCW.close();
					    addcheck=true;
					} catch (IOException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					
					state_domain = "(define (problem move_to_b) (:domain BW4TT) \n";
					//String blocks = OG + "\n" + "block1 block2 block3 block4 - block\n";
					//blocks = "block1 block2 block3 block4 - block\n";
					state_object = "(:objects \n" + 
							"        rooma roomb roomc roomd roome roomf roomg roomh roomi roomj roomk rooml - room\n" + 
							blocks + 
							"        robot1 - robot\n"
							+ "blue red ANY - color" + 
							"    ) \n";
					state_goal = " (:goal\n" + 
							"        (and\n" + 
							"          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))\n" + 
							"        )\n" + 
							"    )\n" + 
							")";
					
					
					if(shareddatausedcountrl ==0)
					{
						
						
						blocks = "block1 block2 block3 block4 - block\n";
						//blocks = "block1 block2 - block\n";
						int calculate1 =0; ///can be used to check how many times an input was accepted
						for (int e=0; e< ts.size(); e++)
						{
							if (!(blocks.contains(ts.get(e))))
							{
								blocks = ts.get(e) +" " +blocks;
								changeobj=true;
								calculate1++;
							}
						}
						
						state_init = SMMStates.get(i);
						state_init = state_init.replace("(_TRUE)", "");
						state_init = state_init.replace("{", "");
						state_init = state_init.replace("}", "");
						state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  \n" + 
								"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
								//"(BLOCK block1) (BLOCK block2) ; (BLOCK block3) \n "
								+ " (at-block block1 roomc) (at-block block2 roomf) (at-block block3 roomj) (at-block block4 roome)" + 
								//+ "(at-block block1 rooma) (at-block block2 rooma)" + 	
								"(ROBOT robot1)  \n"
								//"(ROBOT robot1) \n"
								+ "(free robot1) (visited rooma) (observing rooma robot1) \n"
								+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
								//+ "(color-block block1 blue) (color-block block2 blue) \n"
								//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
								//+ "   \n" + state_init + "\n ) \n";
								+ "  \n"+  " \n" + state_init + addstateobj+ "\n ) \n";
								//+ "  (color-preference robot2 red) \n"+  " (at-robby rooma robot2) \n" + state_init + "\n ) \n";
						System.out.println(state_init);
						
						for (int e=0; e< ts2.size(); e++)
						{
							if (!(state_init.contains(ts2.get(e))) && (ts2.get(e) != " ()") && (!(addstateobj.contains(ts2.get(e)))) )
							{
								addstateobj = ts2.get(e) +" " +addstateobj;
								changeobj=true;
								calculate1++;
							}
						}
						
						
						//roomfcounter=0;
						try {
							//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
							//System.out.println(reader.read());
							
							Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/shareddatausedcount.txt"));
							
							int t = 0;
							while(sc.hasNextInt())
							{
								shareddatausedcount  = sc.nextInt();
							}
							//System.out.println(tll[0]);
							
						} catch (IOException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
						
						
						if (changeobj)
						{
							if (addcheck==false)
							{
							shareddatausedcount++;
							}
							
							state_init = SMMStates.get(i);
							state_init = state_init.replace("(_TRUE)", "");
							state_init = state_init.replace("{", "");
							state_init = state_init.replace("}", "");
							state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  \n" + 
									"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
									//"(BLOCK block1) (BLOCK block2) \n "
									+ " (at-block block1 roomc) (at-block block2 roomf) (at-block block3 roomj) (at-block block4 roome)" + 
									//+ "(at-block block1 rooma) (at-block block2 rooma)" + 	
									"(ROBOT robot1) \n"
									//"(ROBOT robot1) \n"
									+ "(free robot1) (visited rooma) (observing rooma robot1) \n"
									+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
									//+ "(color-block block1 blue) (color-block block2 blue) \n"
									//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
									+ "   \n"+  " \n" + state_init + addstateobj +"\n ) \n";
									//+ "  (color-preference robot2 red) \n"+  " (at-robby rooma robot2)\n" + state_init + addstateobj+ "\n ) \n";
							System.out.println(state_init);
							/*
							state_init = SMMStates.get(i);
							state_init = state_init.replace("(_TRUE)", "");
							state_init = state_init.replace("{", "");
							state_init = state_init.replace("}", "");
							state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) \n" + 
									"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
									+ "(at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma)" + 
									"(ROBOT robot1) (ROBOT robot2) \n"
									+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
									//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
									+ "  (color-preference robot2 red) \n"+  " \n" + state_init+ "\n " + addstateobj + "\n ) \n";
							System.out.println(state_init);*/
						}
						
						try {
							//tll[0]=0;
							myCW = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/shareddatausedcount.txt");
							myCW.write(new Integer(shareddatausedcount).toString());
						    myCW.close();
						    addcheck=true;
						} catch (IOException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
						
						state_domain = "(define (problem move_to_b) (:domain BW4TT) \n";
						//String blocks = OG + "\n" + "block1 block2 block3 block4 - block\n";
						//blocks = "block1 block2 block3 block4 - block\n";
						state_object = "(:objects \n" + 
								"        rooma roomb roomc roomd roome roomf roomg roomh roomi roomj roomk rooml - room\n" + 
								blocks + 
								"        robot1 - robot\n"
								+ "blue red ANY - color" + 
								"    ) \n";
						state_goal = " (:goal\n" + 
								"        (and\n" + 
								"          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))\n" + 
								"        )\n" + 
								"    )\n" + 
								")";
						
						shareddatausedcountrl++;
						try {
							//tll[0]=0;
							myCW = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/shareddatausedcountrobotrelated.txt");
							myCW.write(new Integer(shareddatausedcountrl).toString());
						    myCW.close();
						    addcheck=true;
						} catch (IOException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
					}
					/*String state_init = SMMStates.get(i);
					state_init = state_init.replace("(_TRUE)", "");
					state_init = state_init.replace("{", "");
					state_init = state_init.replace("}", "");
					state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) \n" + 
							"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
							+ "(at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma)" + 
							"(ROBOT robot1) (ROBOT robot2) \n"
							+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
							//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
							+ "  (color-preference robot2 red) \n"+  " \n" + state_init+ ";" + "\n ) \n";
					System.out.println(state_init);*/
					
					
					/*
					String[] arrOfData = null;
					String strOfData ="";
					if (sendData==true)
					{
						int c =0;
						for (int x =0; x< gtt.size();x++)
						{
							String temp = gtt.get(x);
							if (isStringUpperCase(temp))
							{
								System.out.println("Start of Communication2 VIJAYYYYY");
								String test_state_init = state_init;
								String[] arrOfStr = test_state_init.split("[(]");
								
								for (int o=0;o<arrOfStr.length;o++)
								{
									String tp = arrOfStr[o];
									if (tp.contains(temp))
									{
										System.out.println(arrOfStr[c]);
										strOfData = strOfData + arrOfStr[o];
										c++;
										//arrOfData[o] = arrOfStr[o];
									}
									
								}
								
							}
							System.out.println("Start of Communication2 VIJAYYYYY2");
							arrOfData = strOfData.split(" ");
							for (int o=0;o<arrOfData.length;o++)
							{
									String ty = arrOfData[o] ;
									if( (!ty.contains(temp)) )
									{
									System.out.println(arrOfData[o]);	
									}
							}
							
						}
						
						
						System.out.println("Start of Communication2");
						System.out.println(ourplan.get(i));
						System.out.println(state_init);
						System.out.println("End of Communication2");
						sendData=false;
					}
					*/
					
					/*try {
					      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/filename.pddl");
					      myWriter.write( state_domain + state_object + state_init + state_goal);
					      myWriter.close();
					      System.out.println("Successfully wrote to the file.");
					    } catch (IOException e) {
					      System.out.println("An error occurred.");
					      e.printStackTrace();
					    }*/
					//if (SMMStates.get(i) == )
					//System.out.println("YAYYYYYYY");
					//System.out.println(problem.stateToString(node.state));
					//int [] tall = new int [100];
					FileWriter myCWriter;
					try {
						//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
						//System.out.println(reader.read());
						
						Scanner scanner = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt"));
						
						int t = 0;
						while(scanner.hasNextInt())
						{
						     tall[t++] = scanner.nextInt();
						}
						System.out.println(tall[0]);
						if (tall[0] == 1000)
						{
							tall[0] =0;
						}
						else
						{
							
							tall[0] = tall[0] +1;
						}
						myCWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
						myCWriter.write(new Integer(tall[0]).toString());
					    myCWriter.close();
					} catch (IOException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
				    
					
					if (counter==0) {
						
						String strsendData= "";
						String finalstrsendData= "";
						String finalstrsendDatablock= "";
						String[] arrOfData = null;
						String[] arrOfBlock = null;
						String strOfData ="";
						String[] state_init_arr = null;
						
						String[] arrsendData = null;
						state_init_arr = state_init.split("[(]");
						//state_init_arr = state_init.split("[((?=())]");
						
						//System.out.println(Arrays.toString(state_init.split("(?<=:)(?=:)|(?<= )(?=:)|(?<=:)(?= )")));
						//state_init_arr = state_init.split("(?<=()(?=()|(?<= )(?=()|(?<=()(?= )");
						
						if (sendData==true)
						{
							
							
							
							
							int c =0;
							for (int x =0; x< gtt.size();x++)
							{
								String temp = gtt.get(x);
								if (isStringUpperCase(temp))
								{
									System.out.println("Start of Communication2 VIJAYYYYY inner");
									String test_state_init = state_init;
									String[] arrOfStr = test_state_init.split("[(]");
									
									for (int o=0;o<arrOfStr.length;o++)
									{
										String tp = arrOfStr[o];
										if (tp.contains(temp))
										{
											System.out.println(arrOfStr[c]);
											strOfData = strOfData + arrOfStr[o];
											c++;
											//arrOfData[o] = arrOfStr[o];
										}
										
									}
									
								}
								
								
							}
							System.out.println(strOfData);
							System.out.println("Start of Communication2222 VIJAYYYYY2");
							arrOfData = strOfData.split("[)]");
								
							System.out.println(arrOfData);
							int z =0;
							for (int o=0;o<arrOfData.length;o++)
							{
								String ty = arrOfData[o] ;
								for (int x =0; x< gtt.size();x++) {
									String temp = gtt.get(x);
									
									if( ((isStringUpperCase(temp))) )
									{
										for (int k = 0; k<temps2.size();k++)
										{
											if ((ty.contains(temps2.get(k))))
											{
												System.out.println(ty);
												System.out.println("Next Step2");
												System.out.println(arrOfData[o]);
												arrOfData = ArrayUtils.remove(arrOfData, o);
												//o--;
											}
									}
								}
							}
							
							/*for (int x =0; x< gtt.size();x++)
							{
								String temp = gtt.get(x);
								
							for (int o=0;o<arrOfData.length;o++)
							{
									String ty = arrOfData[o] ;
									if( (!ty.contains(temp)) )
									{
									System.out.println(arrOfData[o]);	
									}
							}
							}*/
							
							/*System.out.println("Start of Communication2");
							System.out.println(ourplan.get(i));
							System.out.println(state_init);
							System.out.println("End of Communication2");
							sendData=false;*/
						}
							
							
							int ct =0;
							//int location[] = new int[];
							for (int o=0;o<arrOfData.length;o++)
							{
								String ty = arrOfData[o];
								for (int k = 0; k<temps2.size();k++)
								{
									if ((ty.contains(temps2.get(k))))
									{
										arrOfData = ArrayUtils.remove(arrOfData, o);
										o--;
										//location[location.length+1] = o;
										ct = 1;
									}
									
									
								}
								
							}
							String strOfBlock = "";
							for (int o=0;o<arrOfData.length;o++)
							{
								//System.out.println("Here");
								//System.out.println(arrOfData[o]);
								strOfBlock = strOfBlock + arrOfData[o];
							}
						
							
							arrOfBlock = strOfBlock.split(" ");
							
							for (int o=0;o<arrOfBlock.length;o++)
							{
								
								if (!(arrOfBlock[o].isEmpty()) && (!(arrOfBlock[o].contains("\n"))) && (!(isStringUpperCase(arrOfBlock[o]))))
								{
											//String b = arrOfBlock[o];
											//if (b)
											System.out.println("Herexxx");
											System.out.println(arrOfBlock[o]);
											finalstrsendDatablock = finalstrsendDatablock + arrOfBlock[o] + "\n";
											for (int d=0; d< state_init_arr.length; d++)
											{
												//if (state_init_arr[d].contains(arrOfBlock[o]))
												if (state_init_arr[d].contains(arrOfBlock[o]) || state_init_arr[d].contains("visited"))
												{
													strsendData = strsendData +state_init_arr[d]; 
												}
											}
									
								}
								//strOfBlock = strOfBlock + arrOfData[o];
							}
							System.out.println(finalstrsendDatablock);
							System.out.println(strsendData);
							strsendData.replace("  )", "");
							strsendData.replace("\n ", "");
							//strsendData.replace(" ", "");
							strsendData.replace(" s*", "");
							//System.out.println("Check str data");
							//System.out.println(strsendData);
							strsendData = strsendData.trim();
							//System.out.println(strsendData);
							//System.out.println("end Check str data");
							strsendData = strsendData.replaceAll("[\\n\\t]", "");
							//System.out.println(strsendData);
							strsendData.replace("(carry ", "");
							
							arrsendData = strsendData.split("[)]");
							
							for (int o=0;o<arrsendData.length;o++)
							{
								if (!(arrsendData[o].isEmpty()))
								{
								arrsendData[o] = "(" +arrsendData[o].trim() +")";
								System.out.println(arrsendData[o]);
								finalstrsendData = finalstrsendData + arrsendData[o] +"\n";
								}
								
							}
							System.out.println();
							System.out.println(finalRoomSend);
							System.out.println(finalstrsendDatablock);
							System.out.println(finalstrsendData);
							
							finalstrsendData = finalstrsendData.replace("(:init  ROOM rooma)", "");
							finalstrsendData = finalstrsendData.replace("(visited rooma)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomb)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomc)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomd)", "");
							finalstrsendData = finalstrsendData.replace("(visited roome)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomf)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomg)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomh)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomi)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomj)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomk)", "");
							finalstrsendData = finalstrsendData.replace("(visited rooml)", "");
							try
							{
									FileWriter room = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/R1toR2/RoomData.txt");
									//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
									room.write(finalRoomSend);
									room.close();
							      
									
							      
									FileWriter r_object = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/R1toR2/ObjectSent.txt",false);
									//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
									r_object.write(finalstrsendDatablock);
									r_object.close();
									
									FileWriter st_Data = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/R1toR2/StatePredicatesSent.txt",false);
									//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
									st_Data.write(finalstrsendData);
									st_Data.close();
									
									
									
							}
							catch (Exception e)
							{
								 System.out.println("An error occurred.");
							      e.printStackTrace();
							}
							
						//if (SMMStates.get(i).contains("observing"))	
							//if (ourplan.get(i).contains("observe"))	
						//{
							try {
							      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/MyStateFiles/filename" + tall[0]+".pddl");
								//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
							      myWriter.write( state_domain + state_object + state_init + state_goal);
							      myWriter.close();
							      FileWriter myWriter3 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/TestMyStateFiles/filename" + tall[0]+".pddl");
									//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
								      myWriter3.write( state_domain + state_object + state_init  + state_goal);
								      myWriter3.close();
							      System.out.println("Successfully wrote to the file.");
							    } catch (IOException e) {
							      System.out.println("An error occurred.");
							      e.printStackTrace();
							    }	
							counter++;
						
						      
						
						
						
						
					try {
							
					      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/FinalSolution/finalstepitaration.txt",true);
					      //myWriter.write("\n" +ourplan.get(i) + " \n" + SMMStates.get(i) );
					      myWriter.write("\n" +ourplan.get(i));
					      myWriter.close();
					      System.out.println("Successfully wrote to the file.");
					      //break;
					    } catch (IOException e) {
					      System.out.println("An error occurred.");
					      e.printStackTrace();
					    }
					
					try {
						
					      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/observedexitcheck.txt");
					      myWriter.write("1");
					      myWriter.close();
					      System.out.println("Successfully wrote to the file.");
					      //break;
					    } catch (IOException e) {
					      System.out.println("An error occurred.");
					      e.printStackTrace();
					    }
					    
					    if((ourplan.get(i).contains("drop") ))
						{
						    try {
		                        	Thread.sleep(40);

		                        } catch (InterruptedException e2) {
		                    	// TODO Auto-generated catch block
		                    	e2.printStackTrace();
		                        }
		                        
		                        //return plan;
						}
					return plan;
					}
					//Main temp = new Main();
					//String[] args = new String() {"/Users/vijayanthtummala/Downloads/trydomain.pddl", "/Users/vijayanthtummala/Downloads/aquaplanning-master/filename\" + counter+\".pddl"};
					
					//temp.main(new String[] {"/Users/vijayanthtummala/Downloads/trydomain.pddl", "/Users/vijayanthtummala/Downloads/tryproblem.pddl"});
					//String[] name = {"/Users/vijayanthtummala/Downloads/trydomain.pddl", "/Users/vijayanthtummala/Downloads/tryproblem.pddl"};
					//Main.main(name);
					//String drinks[] = {"/Users/vijayanthtummala/Downloads/trydomain.pddl", "/Users/vijayanthtummala/Downloads/tryproblem.pddl"};
					//Main.main(drinks);
					//ForwardSearchPlanner f = new ForwardSearchPlanner();
					//mainCopy(drinks);
					//plan.appendAtFront("/Users/vijayanthtummala/Downloads/aquaplanning-master/filename" + counter+".pddl");
					
					//break;
					//}
					else {
						
						try {
						      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/MyStateFiles/filename" + tall[0]+".pddl");
						      //FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
						      myWriter.write( state_domain + state_object + state_init + state_goal);
						      myWriter.close();
						      System.out.println("Successfully wrote to the file.");
						    } catch (IOException e) {
						      System.out.println("An error occurred.");
						      e.printStackTrace();
						    }	
						
						try {
							
						      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/FinalSolution/finalstepitaration.txt", true);
						      //myWriter.write("\n" +ourplan.get(i) + " \n" + SMMStates.get(i) + "\n\n" );
						      myWriter.write("\n" +ourplan.get(i));
						      myWriter.close();
						      System.out.println("Successfully wrote to the file.");
						    } catch (IOException e) {
						      System.out.println("An error occurred.");
						      e.printStackTrace();
						    }
						counter++;
						
					}
					
					/*try {
						MySleep();
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}*/
					
					//Thread.sleep(4000);
				}
					//System.out.println("Lets c");
					//System.exit(0);*/
					
					//String args[] = new String[2];
					//args[0]= "/Users/vijayanthtummala/Downloads/trydomain.pddl";
					//args[1]="/Users/vijayanthtummala/Downloads/tryproblem.pddl";
					//Main.main(args);
					//break;
					
					
					// i is the index
				    // yourArrayList.get(i) is the element
				
					}
					}
				else
				{
					
					
					try {
						//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
						//System.out.println(reader.read());
						
						FileWriter myCWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
						myCWriter.write("1000");
					    myCWriter.close();
					} catch (IOException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					
					System.out.println("Else Part");
					
					
					
					/*if (sendData==true)
					{
						
						
					}*/
					//int shareddatausedcount=0;
					int roomfcounter =0;
					boolean tryexit=false;
					outerloop:
					for (int i = 0; i < ourplan.size(); i ++) {
						/*
						if(i>9)
						{
						    return plan;
						}*/
						
						
						System.out.println(tt.size());
						System.out.println("Lets ccccc");
						System.out.println(ourplan.get(i));
						System.out.println(SMMStates.get(i));
						System.out.println();
						for (int j = 0; j<tt.size();j++)
						{
							//tt.get(j);
							if (ourplan.get(i).contains(tt.get(j)))
							{
								//System.out.println("Vijay");
								//System.out.println(j);
								 
								for (int k = 0; k<temps2.size();k++)
								{
									if (ourplan.get(i).contains(temps2.get(k)))
									{
										
										//System.out.println("Vijay2");
										//System.out.println(j);
										System.out.println("Start of Communication");
										System.out.println(ourplan.get(i));
										System.out.println(SMMStates.get(i));
										System.out.println("End of Communication");
										sendData=true;
										finalRoomSend = temps2.get(k);
									}
								}
								
							}
						}
						
						String OG = "";
						String SG = "";
						List<String> ts = new ArrayList<String>();
						List<String> ts2 = new ArrayList<String>();
						List<String> ts2temp = new ArrayList<String>();
						//List<String> allshareddataStored = new ArrayList<String>();
						String blocks;
						String addstateobj="";
						boolean changeobj= false;
						try {
							
						
							

					    // for-each loop

					    // create Scanner inFile1
					    //Scanner if1 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/ObjectSent.txt")).useDelimiter("\n");
							Scanner if1 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/ObjectSent.txt"));

					    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
					    // List<String> temps = new LinkedList<String>();
					    

					    // while loop
					    while (if1.hasNext()) {
					      // find next line
					    	String tp; 
					    	tp = if1.next();
					      OG += tp;
					      ts.add(tp);
					    }
					   // Scanner if2 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/StatePredicatesSent.txt")).useDelimiter("\n");
					    Scanner if2 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/StatePredicatesSent.txt")).useDelimiter(",\\s*");
						    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
						    // List<String> temps = new LinkedList<String>();
						    

						    // while loop
						    while (if2.hasNext()) {
						      // find next line
						    	String tp; 
						    	tp = if2.next();
						      SG += tp;
						      //SG += if2.next();
						      ts2.add(tp);
						    }
					    if2.close();
					    	}
						catch(Exception e)
						{
							
						}
						
						try {
						      File myObj = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/StatePredicatesSent.txt");
						      Scanner myReader = new Scanner(myObj).useDelimiter(",\\s*");
						      while (myReader.hasNextLine()) {
						        String data = myReader.nextLine();
						        System.out.println(data);
						        SG += data;
							      //SG += if2.next();
							      ts2.add(data);
						      }
						      myReader.close();
						    } catch (Exception e) {
						      //System.out.println("An error occurred.");
						      //e.printStackTrace();
						    }
						
						int bdata=0;
						for (int i1 =0; i1<ts2.size();i1++)
						{
							if (ts2.get(i1).contains("()"))
							{
								bdata=1;
							}
							if(!(ts2.get(i1).contains("(")))
							{
								bdata=1;
							}
							if(ts2.get(i1).contains(rtt.get(0)))
							{
								bdata=1;
							}
							if(bdata==0)
							{
								ts2temp.add(ts2.get(i1));
							}
							/*if(!(ts2.get(i1).contains(")")))
							{
								ts2.remove(i1);
							}*/
							bdata=0;
						}
						ts2.clear();
						
						for (int i1 =0; i1<ts2temp.size();i1++)
						{
							ts2.add(ts2temp.get(i1));
							
						}
						/*
						File myObj = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/ObjectSent.txt"); 
					    if (myObj.delete()) { 
					      System.out.println("Deleted the file: " + myObj.getName());
					    } else {
					      System.out.println("Failed to delete the file.");
					    } 
						
					    File myObj1 = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/StatePredicatesSent.txt"); 
					    if (myObj1.delete()) { 
					      System.out.println("Deleted the file: " + myObj1.getName());
					    } else {
					      System.out.println("Failed to delete the file.");
					    } 
						*/
						blocks = "block1 block2 block3 block4 - block\n";
						//blocks = "block1 block2 - block\n";
						int calculate =0; ///can be used to check how many times an input was accepted
						for (int e=0; e< ts.size(); e++)
						{
							if (!(blocks.contains(ts.get(e))))
							{
								blocks = ts.get(e) +" " +blocks;
								changeobj=true;
								calculate++;
							}
						}
						
						state_init = SMMStates.get(i);
						state_init = state_init.replace("(_TRUE)", "");
						state_init = state_init.replace("{", "");
						state_init = state_init.replace("}", "");
						state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  \n" + 
								"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
								//"(BLOCK block1) (BLOCK block2) \n "
								+ " (at-block block1 roomc) (at-block block2 roomf) (at-block block3 roomj) (at-block block4 roome)" + 
								//+ "(at-block block1 rooma) (at-block block2 rooma)" + 	
								"(ROBOT robot1)  \n"
								//"(ROBOT robot1) \n"
								+ "(free robot1) (visited rooma) (observing rooma robot1) \n"
								+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
								//+ "(color-block block1 blue) (color-block block2 blue) \n"
								//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
								//+ "  (color-preference robot2 red) \n"+  " \n" + state_init + "\n ) \n";
								+ "  \n"+  " \n" + state_init + addstateobj +"\n ) \n";
						//System.out.println(state_init);
			
						/*blocks = "block1 block2 block3 block4 - block\n";
						int calculate =0; ///can be used to check how many times an input was accepted
						for (int e=0; e< ts.size(); e++)
						{
							if (!(blocks.contains(ts.get(e))))
							{
								blocks = ts.get(e) +" " +blocks;
								changeobj=true;
								calculate++;
							}
						}
						
						state_init = SMMStates.get(i);
						state_init = state_init.replace("(_TRUE)", "");
						state_init = state_init.replace("{", "");
						state_init = state_init.replace("}", "");
						state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) \n" + 
								"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
								+ "(at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma)" + 
								"(ROBOT robot1) (ROBOT robot2) \n"
								+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
								//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
								+ "  (color-preference robot2 red) \n"+  " \n" + state_init + "\n ) \n";
						System.out.println(state_init);*/
						
						for (int e=0; e< ts2.size(); e++)
						{
							if (!(state_init.contains(ts2.get(e))) && (ts2.get(e) != " ()") && (!(addstateobj.contains(ts2.get(e)))))
							{
								addstateobj = ts2.get(e) +" " +addstateobj;
								changeobj=true;
								calculate++;
							}
						}
						
						
						//roomfcounter=0;
						try {
							//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
							//System.out.println(reader.read());
							
							Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/shareddatausedcount.txt"));
							
							int t = 0;
							while(sc.hasNextInt())
							{
								shareddatausedcount  = sc.nextInt();
							}
							//System.out.println(tll[0]);
							
						} catch (IOException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
						
						
						
						if (changeobj)
						{
							shareddatausedcount++;
							state_init = SMMStates.get(i);
							state_init = state_init.replace("(_TRUE)", "");
							state_init = state_init.replace("{", "");
							state_init = state_init.replace("}", "");
							state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  \n" + 
									"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
									//"(BLOCK block1) (BLOCK block2) \n "
									+ " (at-block block1 roomc) (at-block block2 roomf) (at-block block3 roomj) (at-block block4 roome)" + 
									//+ "(at-block block1 rooma) (at-block block2 rooma)" + 	
									"(ROBOT robot1)  \n"
									//"(ROBOT robot1) \n"
									+ "(free robot1) (visited rooma) (observing rooma robot1) \n"
									+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
									//+ "(color-block block1 blue) (color-block block2 blue) \n"
									//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
									+ "   \n"+  " \n" + state_init + addstateobj + "\n ) \n";
									//+ "  (color-preference robot2 red) \n"+  " \n" + state_init + addstateobj+ "\n ) \n";
							System.out.println(state_init);
							
							/*state_init = SMMStates.get(i);
							state_init = state_init.replace("(_TRUE)", "");
							state_init = state_init.replace("{", "");
							state_init = state_init.replace("}", "");
							state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) \n" + 
									"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
									+ "(at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma)" + 
									"(ROBOT robot1) (ROBOT robot2) \n"
									+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
									//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
									+ "  (color-preference robot2 red) \n"+  " \n" + state_init+ "\n " + addstateobj + "\n ) \n";
							System.out.println(state_init);*/
							
							
							
						}
						
						//FileWriter myCWW;
						roomfcounter=0;
						try {
							//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
							//System.out.println(reader.read());
							
							Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/roomfadddatacheck.txt"));
							
							int t = 0;
							while(sc.hasNextInt())
							{
								roomfcounter  = sc.nextInt();
							}
							//System.out.println(tll[0]);
							
						} catch (IOException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
						/*
						if (ourplan.get(i).contains(" roomf ") && roomfcounter==0 && changeobj)
						{
							tryexit=true;
							roomfcounter++;
							blocks = "block5 "+ blocks;
							state_init = SMMStates.get(i);
							state_init = state_init.replace("(_TRUE)", "");
							state_init = state_init.replace("{", "");
							state_init = state_init.replace("}", "");
							state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) \n" + 
									"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) (BLOCK block5) \n "
									+ "(at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma) (at-block block5 roomf)" + 
									"(ROBOT robot1) (ROBOT robot2) \n"
									//"(ROBOT robot1) \n"
									+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) (color-block block5 blue) \n"
									//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
									+ "  (color-preference robot2 red) \n"+  " \n" + state_init+ "\n " + addstateobj + "\n ) \n";
									//+ "  \n"+  " \n" + state_init+ "\n " + addstateobj + "\n ) \n";
							System.out.println(state_init);
							
							
							
							try {
							    
							    //int tex3= 1;
								FileWriter myCWriter3 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/roomfadddatacheck.txt");
								myCWriter3.write(new Integer(roomfcounter).toString());
							    myCWriter3.close();
							} catch (IOException e1) {
								// TODO Auto-generated catch block
								e1.printStackTrace();
							}
							
						}
						*/
						/*
						if (ourplan.get(i).contains(" roomf ") && roomfcounter==0)
						{
							tryexit = true;
							roomfcounter++;
							blocks = "block5 "+ blocks;
							state_init = SMMStates.get(i);
							state_init = state_init.replace("(_TRUE)", "");
							state_init = state_init.replace("{", "");
							state_init = state_init.replace("}", "");
							state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) \n" + 
									"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) (BLOCK block5) \n "
									+ "(at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma) (at-block block5 roomf)" + 
									"(ROBOT robot1) (ROBOT robot2) \n"
									//"(ROBOT robot1) \n"
									+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) (color-block block5 blue) \n"
									//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
									//+ "  (color-preference robot2 red) \n"+  " \n" + state_init+ "\n " + addstateobj + "\n ) \n";
									+ "  (color-preference robot2 red) \n"+  " \n" + state_init + "\n ) \n";
							//+ "  \n"+  " \n" + state_init + "\n ) \n";
							System.out.println(state_init);
							
							try {
							    
							    //int tex3= 1;
								FileWriter myCWriter3 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/roomfadddatacheck.txt");
								myCWriter3.write(new Integer(roomfcounter).toString());
							    myCWriter3.close();
							} catch (IOException e1) {
								// TODO Auto-generated catch block
								e1.printStackTrace();
							}
						}
						
						*/
						state_domain = "(define (problem move_to_b) (:domain BW4TT) \n";
						//String blocks = OG + "\n" + "block1 block2 block3 block4 - block\n";
						//blocks = "block1 block2 block3 block4 - block\n";
						state_object = "(:objects \n" + 
								"        rooma roomb roomc roomd roome roomf roomg roomh roomi roomj roomk rooml - room\n" + 
								blocks + 
								"        robot1 - robot\n"
								//"        robot1 - robot\n"
								+ "blue red ANY - color" + 
								"    ) \n";
						state_goal = " (:goal\n" + 
								"        (and\n" + 
								"          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))\n" + 
								"        )\n" + 
								"    )\n" + 
								")";
						
						try {
							//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
							//System.out.println(reader.read());
							
							Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/roomfadddatacheck.txt"));
							
							int t = 0;
							while(sc.hasNextInt())
							{
								roomfcounter  = sc.nextInt();
							}
							//System.out.println(tll[0]);
							
						} catch (IOException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
						/*
						if(roomfcounter>0 && tryexit==false)
						{
							//roomfcounter++;
							blocks = "block5 "+ blocks;
							state_init = SMMStates.get(i);
							state_init = state_init.replace("(_TRUE)", "");
							state_init = state_init.replace("{", "");
							state_init = state_init.replace("}", "");
							state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  \n" + 
									"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) (BLOCK block5) \n "
									+ "(at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma) (at-block block5 roomf)" + 
									"(ROBOT robot1) \n"
									//"(ROBOT robot1) \n"
									+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) (color-block block5 blue) \n"
									//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
									+ "   \n"+  " \n" + state_init+ "\n " + addstateobj + "\n ) \n";
									//+ "   \n"+  " \n" + state_init+ "\n " + addstateobj + "\n ) \n";
							System.out.println(state_init);
							
						try {
							int temp =1;
							FileWriter myCWriter3 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/roomfadddatacheckExit.txt");
							myCWriter3.write(new Integer(temp).toString());
						    myCWriter3.close();
						} catch (IOException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
						
						}*/
						
						
						
						
						/*String state_init = SMMStates.get(i);
						state_init = state_init.replace("(_TRUE)", "");
						state_init = state_init.replace("{", "");
						state_init = state_init.replace("}", "");
						state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) \n" + 
								"(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n "
								+ "(at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma)" + 
								"(ROBOT robot1) (ROBOT robot2) \n"
								+ "(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) \n"
								//+ "  (color-preference robot2 red) \n"+ SG+ " \n" + state_init+ ";" + ourplan.get(i) + "\n ) \n";
								+ "  (color-preference robot2 red) \n"+  " \n" + state_init+ ";" + "\n ) \n";
						System.out.println(state_init);*/
						
						
						/*
						String[] arrOfData = null;
						String strOfData ="";
						if (sendData==true)
						{
							int c =0;
							for (int x =0; x< gtt.size();x++)
							{
								String temp = gtt.get(x);
								if (isStringUpperCase(temp))
								{
									System.out.println("Start of Communication2 VIJAYYYYY");
									String test_state_init = state_init;
									String[] arrOfStr = test_state_init.split("[(]");
									
									for (int o=0;o<arrOfStr.length;o++)
									{
										String tp = arrOfStr[o];
										if (tp.contains(temp))
										{
											System.out.println(arrOfStr[c]);
											strOfData = strOfData + arrOfStr[o];
											c++;
											//arrOfData[o] = arrOfStr[o];
										}
										
									}
									
								}
								System.out.println("Start of Communication2 VIJAYYYYY2");
								arrOfData = strOfData.split(" ");
								for (int o=0;o<arrOfData.length;o++)
								{
										String ty = arrOfData[o] ;
										if( (!ty.contains(temp)) )
										{
										System.out.println(arrOfData[o]);	
										}
								}
								
							}
							
							
							System.out.println("Start of Communication2");
							System.out.println(ourplan.get(i));
							System.out.println(state_init);
							System.out.println("End of Communication2");
							sendData=false;
						}
						*/
						
						/*try {
						      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/filename.pddl");
						      myWriter.write( state_domain + state_object + state_init + state_goal);
						      myWriter.close();
						      System.out.println("Successfully wrote to the file.");
						    } catch (IOException e) {
						      System.out.println("An error occurred.");
						      e.printStackTrace();
						    }*/
						//if (SMMStates.get(i) == )
						//System.out.println("YAYYYYYYY");
						//System.out.println(problem.stateToString(node.state));
						//int [] tall = new int [100];
						FileWriter myCWriter;
						try {
							//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
							//System.out.println(reader.read());
							
							Scanner scanner = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt"));
							
							int t = 0;
							while(scanner.hasNextInt())
							{
							     tall[t++] = scanner.nextInt();
							}
							System.out.println(tall[0]);
							if (tall[0] == 1000)
							{
								tall[0] =0;
							}
							else
							{
								
								tall[0] = tall[0] +1;
							}
							myCWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
							myCWriter.write(new Integer(tall[0]).toString());
						    myCWriter.close();
						} catch (IOException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
					    
						
						String strsendData= "";
						String finalstrsendData= "";
						String finalstrsendDatablock= "";
						String[] arrOfData = null;
						String[] arrOfBlock = null;
						String strOfData ="";
						String[] state_init_arr = null;
						
						String[] arrsendData = null;
						state_init_arr = state_init.split("[(]");
						//state_init_arr = state_init.split("[((?=())]");
						
						//System.out.println(Arrays.toString(state_init.split("(?<=:)(?=:)|(?<= )(?=:)|(?<=:)(?= )")));
						//state_init_arr = state_init.split("(?<=()(?=()|(?<= )(?=()|(?<=()(?= )");
						
						if (sendData==true)
						{
							
							
							
							
							int c =0;
							for (int x =0; x< gtt.size();x++)
							{
								String temp = gtt.get(x);
								if (isStringUpperCase(temp))
								{
									System.out.println("Start of Communication2 VIJAYYYYY inner");
									String test_state_init = state_init;
									String[] arrOfStr = test_state_init.split("[(]");
									
									for (int o=0;o<arrOfStr.length;o++)
									{
										String tp = arrOfStr[o];
										if (tp.contains(temp))
										{
											System.out.println(arrOfStr[c]);
											strOfData = strOfData + arrOfStr[o];
											c++;
											//arrOfData[o] = arrOfStr[o];
										}
										
									}
									
								}
								
								
							}
							System.out.println(strOfData);
							System.out.println("Start of Communication2222 VIJAYYYYY2");
							arrOfData = strOfData.split("[)]");
								
							System.out.println(arrOfData);
							int z =0;
							for (int o=0;o<arrOfData.length;o++)
							{
								String ty = arrOfData[o] ;
								for (int x =0; x< gtt.size();x++) {
									String temp = gtt.get(x);
									
									if( ((isStringUpperCase(temp))) )
									{
										for (int k = 0; k<temps2.size();k++)
										{
											if ((ty.contains(temps2.get(k))))
											{
												System.out.println(ty);
												System.out.println("Next Step2");
												System.out.println(arrOfData[o]);
												arrOfData = ArrayUtils.remove(arrOfData, o);
												//o--;
											}
									}
								}
							}
							
							/*for (int x =0; x< gtt.size();x++)
							{
								String temp = gtt.get(x);
								
							for (int o=0;o<arrOfData.length;o++)
							{
									String ty = arrOfData[o] ;
									if( (!ty.contains(temp)) )
									{
									System.out.println(arrOfData[o]);	
									}
							}
							}*/
							
							/*System.out.println("Start of Communication2");
							System.out.println(ourplan.get(i));
							System.out.println(state_init);
							System.out.println("End of Communication2");
							sendData=false;*/
						}
							
							
							int ct =0;
							//int location[] = new int[];
							for (int o=0;o<arrOfData.length;o++)
							{
								String ty = arrOfData[o];
								for (int k = 0; k<temps2.size();k++)
								{
									if ((ty.contains(temps2.get(k))))
									{
										arrOfData = ArrayUtils.remove(arrOfData, o);
										o--;
										//location[location.length+1] = o;
										ct = 1;
									}
									
									
								}
								
							}
							String strOfBlock = "";
							for (int o=0;o<arrOfData.length;o++)
							{
								//System.out.println("Here");
								//System.out.println(arrOfData[o]);
								strOfBlock = strOfBlock + arrOfData[o];
							}
						
							
							arrOfBlock = strOfBlock.split(" ");
							
							for (int o=0;o<arrOfBlock.length;o++)
							{
								
								if (!(arrOfBlock[o].isEmpty()) && (!(arrOfBlock[o].contains("\n"))) && (!(isStringUpperCase(arrOfBlock[o]))))
								{
											//String b = arrOfBlock[o];
											//if (b)
											System.out.println("Herexxx");
											System.out.println(arrOfBlock[o]);
											finalstrsendDatablock = finalstrsendDatablock + arrOfBlock[o] + "\n";
											for (int d=0; d< state_init_arr.length; d++)
											{
												//if (state_init_arr[d].contains(arrOfBlock[o]))
												if (state_init_arr[d].contains(arrOfBlock[o]) || state_init_arr[d].contains("visited"))
												{
													strsendData = strsendData +state_init_arr[d]; 
												}
											}
									
								}
								//strOfBlock = strOfBlock + arrOfData[o];
							}
							System.out.println(finalstrsendDatablock);
							System.out.println(strsendData);
							strsendData.replace("  )", "");
							strsendData.replace("\n ", "");
							//strsendData.replace(" ", "");
							strsendData.replace(" s*", "");
							//System.out.println("Check str data");
							//System.out.println(strsendData);
							strsendData = strsendData.trim();
							//System.out.println(strsendData);
							//System.out.println("end Check str data");
							strsendData = strsendData.replaceAll("[\\n\\t]", "");
							//System.out.println(strsendData);
							strsendData = strsendData.replace("(carry ", "");
							
							arrsendData = strsendData.split("[)]");
							
							for (int o=0;o<arrsendData.length;o++)
							{
								if (!(arrsendData[o].isEmpty()))
								{
								arrsendData[o] = "(" +arrsendData[o].trim() +")";
								System.out.println(arrsendData[o]);
								finalstrsendData = finalstrsendData + arrsendData[o] +"\n";
								}
								
							}
							System.out.println();
							System.out.println(finalRoomSend);
							System.out.println(finalstrsendDatablock);
							System.out.println(finalstrsendData);
							
							finalstrsendData = finalstrsendData.replace("(:init  ROOM rooma)", "");
							finalstrsendData = finalstrsendData.replace("(visited rooma)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomb)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomc)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomd)", "");
							finalstrsendData = finalstrsendData.replace("(visited roome)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomf)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomg)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomh)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomi)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomj)", "");
							finalstrsendData = finalstrsendData.replace("(visited roomk)", "");
							finalstrsendData = finalstrsendData.replace("(visited rooml)", "");
							try
							{
									FileWriter room = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/R1toR2/RoomData.txt");
									//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
									room.write(finalRoomSend);
									room.close();
							      
									FileWriter r_object = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/R1toR2/ObjectSent.txt",false);
									//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
									r_object.write(finalstrsendDatablock);
									r_object.close();
									
									FileWriter st_Data = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/R1toR2/StatePredicatesSent.txt",false);
									//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
									st_Data.write(finalstrsendData);
									st_Data.close();
									
									
									
							}
							catch (Exception e)
							{
								 System.out.println("An error occurred.");
							      e.printStackTrace();
							}
							
							
						      
						try {
						      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/MyStateFiles/filename" + tall[0]+".pddl");
							//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
						      myWriter.write( state_domain + state_object + state_init + state_goal);
						      myWriter.close();
						      FileWriter myWriter3 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/TestMyStateFiles/filename" + tall[0]+".pddl");
								//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
							      myWriter3.write( state_domain + state_object + state_init  + state_goal);
							      myWriter3.close();
						      System.out.println("Successfully wrote to the file.");
						    } catch (IOException e) {
						      System.out.println("An error occurred.");
						      e.printStackTrace();
						    }	
						
						
						
					try {
							
					      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/FinalSolution/finalstepitaration.txt",true);
					      //myWriter.write("\n" +ourplan.get(i) + " \n" + SMMStates.get(i) );
					      myWriter.write("\n" +ourplan.get(i));
					      myWriter.close();
					      System.out.println("Successfully wrote to the file.");
					      //break;
					    } catch (IOException e) {
					      System.out.println("An error occurred.");
					      e.printStackTrace();
					    }
					
					if((ourplan.get(i).contains("drop") ))
						{
						    try {
		                        	Thread.sleep(40);

		                        } catch (InterruptedException e2) {
		                    	// TODO Auto-generated catch block
		                    	e2.printStackTrace();
		                        }
		                        
		                        //return plan;
						}
						
					//Main temp = new Main();
					//String[] args = new String() {"/Users/vijayanthtummala/Downloads/trydomain.pddl", "/Users/vijayanthtummala/Downloads/aquaplanning-master/filename\" + counter+\".pddl"};
					
					//temp.main(new String[] {"/Users/vijayanthtummala/Downloads/trydomain.pddl", "/Users/vijayanthtummala/Downloads/tryproblem.pddl"});
					//String[] name = {"/Users/vijayanthtummala/Downloads/trydomain.pddl", "/Users/vijayanthtummala/Downloads/tryproblem.pddl"};
					//Main.main(name);
					//String drinks[] = {"/Users/vijayanthtummala/Downloads/trydomain.pddl", "/Users/vijayanthtummala/Downloads/tryproblem.pddl"};
					//Main.main(drinks);
					//ForwardSearchPlanner f = new ForwardSearchPlanner();
					//mainCopy(drinks);
					//plan.appendAtFront("/Users/vijayanthtummala/Downloads/aquaplanning-master/filename" + counter+".pddl");
					counter++;
					//break;
					}
						else {
							
							try {
							      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/MyStateFiles/filename" + tall[0]+".pddl");
							      //FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
							      myWriter.write( state_domain + state_object + state_init + state_goal);
							      myWriter.close();
							      System.out.println("Successfully wrote to the file.");
							    } catch (IOException e) {
							      System.out.println("An error occurred.");
							      e.printStackTrace();
							    }	
							
							try {
								
							      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/FinalSolution/finalstepitaration.txt", true);
							      //myWriter.write("\n" +ourplan.get(i) + " \n" + SMMStates.get(i) + "\n\n" );
							      myWriter.write("\n" +ourplan.get(i));
							      myWriter.close();
							      System.out.println("Successfully wrote to the file.");
							    } catch (IOException e) {
							      System.out.println("An error occurred.");
							      e.printStackTrace();
							    }
							counter++;
							
						}	
						
						
						/*
						try {
						      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/MyStateFiles/filename" + tall[0]+".pddl");
							//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
						      myWriter.write( state_domain + state_object + state_init + state_goal);
						      myWriter.close();
						      FileWriter myWriter3 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/TestMyStateFiles/filename" + tall[0]+".pddl");
								//FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/tryproblem.pddl");
							      myWriter3.write( state_domain + state_object + state_init  + state_goal);
							      myWriter3.close();
						      System.out.println("Successfully wrote to the file.");
						    } catch (IOException e) {
						      System.out.println("An error occurred.");
						      e.printStackTrace();
						    }	
						
					
						
					try {
							
					      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/FinalSolution/finalstepitaration.txt",true);
					      myWriter.write("\n" +ourplan.get(i) + " \n" + SMMStates.get(i) );
					      myWriter.close();
					      System.out.println("Successfully wrote to the file.");
					      //break;
					    } catch (IOException e) {
					      System.out.println("An error occurred.");
					      e.printStackTrace();
					    }
					
					
						
						
						System.out.println(ourplan.get(i));
						System.out.println(SMMStates.get(i));
						
						System.out.println(tt.size());
						for (int j = 0; j<tt.size();j++)
						{
							//tt.get(j);
							if (ourplan.get(i).contains(tt.get(j)))
							{
								//System.out.println("Vijay");
								//System.out.println(j);
								 
								for (int k = 0; k<temps2.size();k++)
								{
									if (ourplan.get(i).contains(temps2.get(k)))
									{
										//System.out.println("Vijay2");
										//System.out.println(j);
										//System.out.println("Else Start of Communication");
										//System.out.println(ourplan.get(i));
										//System.out.println(SMMStates.get(i));
										//System.out.println("Else End of Communication");
										sendData=true;
									}
								}
								
							}
						}
						if (sendData==true)
						{
							System.out.println("Start of Communication2");
							System.out.println(ourplan.get(i));
							//System.out.println(state_init);
							System.out.println(SMMStates.get(i));
							System.out.println("End of Communication2");
							sendData=false;
						}
						*/
						
						if(roomfcounter>0)
						{
							
							break outerloop;
						}
						
					}
					
					try {
						tll[0]=0;
						myCW = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/check.txt");
						myCW.write(new Integer(tll[0]).toString());
					    myCW.close();
					} catch (IOException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					
					
					int roomfcounterexit=0;
					try {
						//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
						//System.out.println(reader.read());
						
						Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/roomfadddatacheck.txt"));
						
						int t = 0;
						while(sc.hasNextInt())
						{
							roomfcounterexit  = sc.nextInt();
						}
						//System.out.println(tll[0]);
						
					} catch (IOException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					
					try {
						//tll[0]=0;
						myCW = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/shareddatausedcount.txt");
						myCW.write(new Integer(shareddatausedcount).toString());
					    myCW.close();
					} catch (IOException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					//allshareddataStored
					
					
					
					
					if(roomfcounter==0)
					{
						System.exit(0);
					}
					
					return plan;
					/*System.exit(0);
					File directory = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/MyStateFiles");

					// Get all files in directory

					File[] files = directory.listFiles();
					for (File file : files)
					{
					   // Delete each file

					   if (!file.delete())
					   {
					       // Failed to delete file

					       System.out.println("Failed to delete "+file);
					   }
					} */
				}
				System.out.println("end ourplan");
				//String drinks[] = {"/Users/vijayanthtummala/Downloads/trydomain.pddl", "/Users/vijayanthtummala/Downloads/aquaplanning-master/filename" + 0+ ".pddl"};
				//Main.main(drinks);
				//ForwardSearchPlanner f = new ForwardSearchPlanner();
				//mainCopy(drinks);
				//System.out.println("Yay");
				long timeStop = System.nanoTime();
				Logger.log(Logger.INFO, "Visited " + iteration + " nodes in total. "
						+ "Search time: " + (timeStop - timeStart)/1000000 + "ms");
				
				
				int roomfcounterexit=0;
				try {
					//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master/counter.txt");
					//System.out.println(reader.read());
					
					Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/roomfadddatacheck.txt"));
					
					int t = 0;
					while(sc.hasNextInt())
					{
						roomfcounterexit  = sc.nextInt();
					}
					//System.out.println(tll[0]);
					
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				if (roomfcounterexit==1)
				{
					for (int i = 0; i < ourplan.size(); i ++) {
						
						//System.out.println("Vijay Test");
						//System.out.println(ourplan.get(i));
						//System.out.println(SMMStates.get(i));
						try {
							
						      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/FinalSolution/finalstepitaration.txt", true);
						      //myWriter.write("\n" +ourplan.get(i) + " \n" + SMMStates.get(i) + "\n\n" );
						      myWriter.write("\n" +ourplan.get(i));
						      myWriter.close();
						      System.out.println("Successfully wrote to the file.");
						    } catch (IOException e) {
						      System.out.println("An error occurred.");
						      e.printStackTrace();
						    }
					}
				}
				return plan;
			}
			
			// Expand node: iterate over operators
			for (Action action : aindex.getApplicableActions(node.state)) {
				// Create new node by applying the operator
				//Important steps next 2
				//System.out.println(action.getName());
				//System.out.println(problem.stateToString(newState));
				//System.out.println(action.getName());
				
				/*for (int i =0;i<size;i++){
			        //System.out.println(IP[i]);
					if(action.getName().contains(IP[i]))
					{
						//System.out.println("IP Matched");
						//System.out.println(IP[i]);
						//System.out.println(action.getName());
					}
				
				}*/
				//System.out.println(action.getEffectsPos());
				// Atom atom = new Atom(, "Helloooo",true);
				//System.out.println(atom.getId());
				//System.out.println();
				//State st = new State(atom);
				//node.state.set(atom);
				
				//node.state.addAll(am);
				//State try = new State();
				
				/*Atom atomz = new Atom(80, "(at-block block5 rooma)",true);
				List<Atom> initialStateAtomsz = new ArrayList<>();
				initialStateAtomsz.add(atomz);
				//System.out.println(initialStateAtoms);
				AtomSet amz = new AtomSet(initialStateAtoms);
				//State stt = new State(initialStateAtoms);
				//stt.addAll(am);
				node.state.addAll(amz);*/
				
				//System.out.println(problem.stateToString(node.state));
				State newState = action.apply(node.state); //important pre existing line of code
				
				
				
				//System.out.println(newState.getAtomSet().toString());
				//problem.setInitialState(newState);
				//System.out.println(problem.stateToString(newState));
				//problem.setInitialState(newState);
				//State temp =problem.getInitialState(); 
				
				
				//System.out.println(problem.stateToString(temp));
				//action.
				//State initialState = getInitialState(newState, true);
				
				//System.out.println(problem.getNumericAtomNames());
				//System.out.println(problem.getAtomNames());
				//System.out.println(action.getName());
				//System.out.println(problem.stateToString(newState));
				//System.out.println(action.getName());
				//System.out.println(problem.stateToString(newState));
				
				//System.out.println(SMMactions);
				//System.out.println(SMMStates);
				//System.out.println(node.state);
				//System.out.println(problem.stateToString(newState));
				SMMactions.add(action.getName());
				SMMStates.add(problem.stateToString(newState));
				// Add new node to frontier
				SearchNode newNode = new SearchNode(node, newState);
				newNode.lastAction = action;
				//System.out.println(action);
				frontier.add(newNode);
			}
			
			iteration++;
			
			// Print amount of visited nodes and search speed
			if ((iteration << visitedNodesPrintInterval) == 0) {
				double elapsedMillis = ((System.nanoTime() - timeStart) * 0.001 * 0.001);
				int nodesPerSecond = (int) (iteration / (0.001 * elapsedMillis));
				Logger.log(Logger.INFO, "Visited " + iteration + " nodes. (" + nodesPerSecond + " nodes/s)");
				visitedNodesPrintInterval--;
			}
		}
		
		// Failure to find a plan within the maximum amount of iterations
		// (or the problem is unsolvable and search space is exhausted)
		if (frontier.isEmpty()) {
			Logger.log(Logger.INFO, "Search space exhausted.");
		} else {
			Logger.log(Logger.INFO, "Interrupted and/or computational resources exhausted.");
		}
		long timeStop = System.nanoTime();
		Logger.log(Logger.INFO, "Visited " + iteration + " nodes in total. Search time: " 
				+ (timeStop - timeStart)/1000000 + "ms");
		return null;
	}


	
	
	
	
	
	
	
	
	private void MySleep() throws InterruptedException {
		// TODO Auto-generated method stub
		Thread.sleep(4000);
	}


	public Plan findPlanAgain(GroundPlanningProblem problem) {
		startSearch();
		//Start of My Code
		//System.out.println("Pleaaaaseeeeeeeeeee");
		ArrayList<String> SMMactions=new ArrayList<String>(); 
		ArrayList<String> SMMStates=new ArrayList<String>();
		ArrayList<String> ourplan=new ArrayList<String>();
		 String[] IP = {"move ", "move2 ", "pickup ", "drop "};
	        int size = IP.length;
	        //for (int i =0;i<size;i++)
	        //System.out.println(IP[i]);
		//End of My Code
		/*System.out.println(problem.getGoal());
		
		Atom atomv1 = new Atom(60, "(visited rooma)",true);
		Atom atomv2 = new Atom(61, "(visited roomb)",true);
		Atom atomv3 = new Atom(62, "(visited roomc)",true);
		Atom atomv4 = new Atom(63, "(visited roomd)",true);
		Atom atomv5 = new Atom(64, "(visited roome)",true);
		Atom atomv6 = new Atom(64, "(visited roomf)",true);
		Atom atom1 = new Atom(70, "(at-block block1 rooma)",true);
		Atom atom2 = new Atom(71, "(at-block block2 rooma)",true);
		Atom atom3 = new Atom(72, "(at-block block3 rooma)",true);
		Atom atom4 = new Atom(73, "(at-block block4 rooma)",true);
		Atom atom5 = new Atom(74, "(at-block block5 rooma)",true);
		List<Atom> initialStateAtoms = new ArrayList<>();
		initialStateAtoms.add(atomv1);
		initialStateAtoms.add(atomv2);
		initialStateAtoms.add(atomv3);
		initialStateAtoms.add(atomv4);
		initialStateAtoms.add(atomv5);
		initialStateAtoms.add(atomv6);
		initialStateAtoms.add(atom1);
		initialStateAtoms.add(atom2);
		initialStateAtoms.add(atom3);
		initialStateAtoms.add(atom4);
		//initialStateAtoms.add(atom5);
		Goal g = new Goal(initialStateAtoms);
		problem.setGoal(g);
		System.out.println(problem.getGoal());*/
		Logger.log(Logger.INFO, "Starting forward search with " + config.toString());
		// Important objects from the planning problem
		
		//Atom atom2 = new Atom(84, "(at-block block5 rooma)",true);
		//List<Atom> initialStateAtoms = new ArrayList<>();
		//initialStateAtoms.add(atom2);
		//System.out.println(initialStateAtoms);
		//AtomSet am = new AtomSet(initialStateAtoms);
		//State st = new State(initialStateAtoms);
		//st.addAll(am);
		//atomsl.set(atom2);
		//System.out.println(atom.getId());
		//System.out.println();
		//State st = problem.getInitialState();
		//System.out.println(problem.stateToString(st));
		//problem.setInitialState(s);
		//st.set(atom2);
		//System.out.println(problem.stateToString(st));
		//problem.setInitialState(st);
		//System.out.println(problem.stateToString(st));
		
		State initState = problem.getInitialState();
		System.out.println(initState);
		
		//initState.set(atom);
		//System.out.println("hereee");
		System.out.println(problem.stateToString(initState));
		System.out.println(problem.getInitialState().toString());
		//System.out.println(initState);
		Goal goal = problem.getGoal();
		System.out.println(goal);
		ActionIndex aindex = new ActionIndex(problem);
		System.out.println(aindex);
		// Initialize forward search
		SearchQueue frontier;
		SearchStrategy strategy = new SearchStrategy(config);
		if (strategy.isHeuristical()) {
			Heuristic heuristic = Heuristic.getHeuristic(problem, config);
			frontier = new SearchQueue(strategy, heuristic);
		} else {
			frontier = new SearchQueue(strategy);
		}
		frontier.add(new SearchNode(null, initState));
		
		int iteration = 1;
		int visitedNodesPrintInterval = 28;
		long timeStart = System.nanoTime();
		
		//System.out.println(problem.getAtomNames());
		
		while (withinComputationalBounds(iteration) && !frontier.isEmpty()) {
			
			// Visit node (by the heuristic provided to the priority queue)
			SearchNode node = frontier.get();
			
			// Is the goal reached?
			if (goal.isSatisfied(node.state)) {
				
				// Extract plan
				Plan plan = new Plan();
				
				
				while (node != null && node.lastAction != null) {
					plan.appendAtFront(node.lastAction);
					ourplan.add(node.lastAction.getName());
					SMMStates.add(problem.stateToString(node.state));
					System.out.println(node.lastAction.getName());
					System.out.println(problem.stateToString(node.state));
					
					/*String state_domain = "(define (problem move_to_b) (:domain BW4TT) \n";
					String state_object = "(:objects \n" + 
							"        rooma roomb roomc roomd roome roomf - room\n" + 
							"        block1 block2 block3 block4 - block\n" + 
							"        robot1 robot2 - robot\n" + 
							"    ) \n";
					String state_goal = " (:goal\n" + 
							"        (and\n" + 
							"          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))\n" + 
							"        )\n" + 
							"    )\n" + 
							")";
					String state_init = problem.stateToString(node.state);
					state_init = state_init.replace("(_TRUE)", "");
					state_init = state_init.replace("{", "");
					state_init = state_init.replace("}", "");
					state_init = "(:init \n (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) \n" + 
							"        (BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) \n" + 
							"        (ROBOT robot1) (ROBOT robot2) \n" + state_init+ "\n ) \n";
					try {
					      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/filename.pddl");
					      myWriter.write( state_domain + state_object + state_init + state_goal);
					      myWriter.close();
					      System.out.println("Successfully wrote to the file.");
					    } catch (IOException e) {
					      System.out.println("An error occurred.");
					      e.printStackTrace();
					    }
					try {
					      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master/finalstepitaration.txt");
					      myWriter.write( node.lastAction.getName() + " \n" + problem.stateToString(node.state) );
					      myWriter.close();
					      System.out.println("Successfully wrote to the file.");
					    } catch (IOException e) {
					      System.out.println("An error occurred.");
					      e.printStackTrace();
					    }
					//System.out.println("Lets c");
					//System.exit(0);*/
					
					
					//System.out.println(node.state);
					//node.state =problem.stateToString(node.state);
					//problem.setInitialState(node.state);
					node = node.parent;
				}
				
				Collections.reverse(ourplan);
				Collections.reverse(SMMactions);
				Collections.reverse(SMMStates);
				//System.out.println(SMMactions);
				//System.out.println(SMMStates);
				//System.out.println(ourplan);
				System.out.println("ourplan");
				System.out.println(ourplan.size());
				int counter =0;
				
				System.out.println("end ourplan");
				String drinks[] = {"/Users/vijayanthtummala/Downloads/trydomain.pddl", "/Users/vijayanthtummala/Downloads/aquaplanning-master/filename" + 0+ ".pddl"};
				//Main.main(drinks);
				//ForwardSearchPlanner f = new ForwardSearchPlanner();
				//mainCopy(drinks);
				//System.out.println("Yay");
				long timeStop = System.nanoTime();
				Logger.log(Logger.INFO, "Visited " + iteration + " nodes in total. "
						+ "Search time: " + (timeStop - timeStart)/1000000 + "ms");
				
				System.out.println("we are in findplanagain");
				return plan;
			}
			
			// Expand node: iterate over operators
			for (Action action : aindex.getApplicableActions(node.state)) {
				// Create new node by applying the operator
				//Important steps next 2
				//System.out.println(action.getName());
				//System.out.println(problem.stateToString(newState));
				//System.out.println(action.getName());
				
				for (int i =0;i<size;i++){
			        //System.out.println(IP[i]);
					if(action.getName().contains(IP[i]))
					{
						//System.out.println("IP Matched");
						//System.out.println(IP[i]);
						//System.out.println(action.getName());
					}
				
				}
				//System.out.println(action.getEffectsPos());
				// Atom atom = new Atom(, "Helloooo",true);
				//System.out.println(atom.getId());
				//System.out.println();
				//State st = new State(atom);
				//node.state.set(atom);
				
				//node.state.addAll(am);
				//State try = new State();
				
				/*Atom atomz = new Atom(80, "(at-block block5 rooma)",true);
				List<Atom> initialStateAtomsz = new ArrayList<>();
				initialStateAtomsz.add(atomz);
				//System.out.println(initialStateAtoms);
				AtomSet amz = new AtomSet(initialStateAtoms);
				//State stt = new State(initialStateAtoms);
				//stt.addAll(am);
				node.state.addAll(amz);*/
				
				//System.out.println(problem.stateToString(node.state));
				State newState = action.apply(node.state); //important pre existing line of code
				
				
				
				//System.out.println(newState.getAtomSet().toString());
				//problem.setInitialState(newState);
				//System.out.println(problem.stateToString(newState));
				//problem.setInitialState(newState);
				//State temp =problem.getInitialState(); 
				
				
				//System.out.println(problem.stateToString(temp));
				//action.
				//State initialState = getInitialState(newState, true);
				
				//System.out.println(problem.getNumericAtomNames());
				//System.out.println(problem.getAtomNames());
				//System.out.println(action.getName());
				//System.out.println(problem.stateToString(newState));
				//System.out.println(action.getName());
				//System.out.println(problem.stateToString(newState));
				
				//System.out.println(SMMactions);
				//System.out.println(SMMStates);
				//System.out.println(node.state);
				//System.out.println(problem.stateToString(newState));
				SMMactions.add(action.getName());
				SMMStates.add(problem.stateToString(newState));
				// Add new node to frontier
				SearchNode newNode = new SearchNode(node, newState);
				newNode.lastAction = action;
				//System.out.println(action);
				frontier.add(newNode);
			}
			
			iteration++;
			
			// Print amount of visited nodes and search speed
			if ((iteration << visitedNodesPrintInterval) == 0) {
				double elapsedMillis = ((System.nanoTime() - timeStart) * 0.001 * 0.001);
				int nodesPerSecond = (int) (iteration / (0.001 * elapsedMillis));
				Logger.log(Logger.INFO, "Visited " + iteration + " nodes. (" + nodesPerSecond + " nodes/s)");
				visitedNodesPrintInterval--;
			}
		}
		
		// Failure to find a plan within the maximum amount of iterations
		// (or the problem is unsolvable and search space is exhausted)
		if (frontier.isEmpty()) {
			Logger.log(Logger.INFO, "Search space exhausted.");
		} else {
			Logger.log(Logger.INFO, "Interrupted and/or computational resources exhausted.");
		}
		long timeStop = System.nanoTime();
		Logger.log(Logger.INFO, "Visited " + iteration + " nodes in total. Search time: " 
				+ (timeStop - timeStart)/1000000 + "ms");
		return null;
	}	


}