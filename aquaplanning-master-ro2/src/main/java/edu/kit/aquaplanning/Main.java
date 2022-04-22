package edu.kit.aquaplanning;


import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.util.ArrayList;
import java.util.Date;

import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.InetAddress;
import java.util.List;
import java.util.Scanner;
import java.io.*;
import java.nio.charset.StandardCharsets;



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


public class Main {
	Process mProcess;
	public void runScript(){
	       Process process;
	       try{
	             process = Runtime.getRuntime().exec(new String[]{"/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/PDDLexec.command","/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/trydomain.pddl","/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/tryproblem.pddl"});
	             mProcess = process;
	       }catch(Exception e) {
	          System.out.println("Exception Raised" + e.toString());
	       }
	       InputStream stdout = mProcess.getInputStream();
	       BufferedReader reader = new BufferedReader(new InputStreamReader(stdout,StandardCharsets.UTF_8));
	       String line;
	       try{
	          while((line = reader.readLine()) != null){
	               System.out.println("stdout: "+ line);
	          }
	       }catch(IOException e){
	             System.out.println("Exception in reading output"+ e.toString());
	       }
	}
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
	
	
	private static void mcall2(String[] args) throws IOException {
		
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
				//plan = planner.findPlan(planningProblem);
				plan = planner.findPlanAgain(planningProblem);
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
				System.out.println("hello");
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
		//args[0]="/Users/vijayanthtummala/Downloads/trydomain.pddl";
		//args[1]="/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/MyStateFiles/filename18.pddl";
		
		//String command = "python -B /Users/vijayanthtummala/Downloads/pddl-parser-master/PDDL.py";
		//String param = "/Users/vijayanthtummala/Downloads/pddl-parser-master/trydomain.pddl /Users/vijayanthtummala/Downloads/pddl-parser-master/tryproblem.pddl";
		//Process p = Runtime.getRuntime().exec(command + param );
		
		try
		{
			
	
		
		// create token1
	    String token1 = "";

	    // for-each loop for calculating heat index of May - October

	    // create Scanner inFile1
	    Scanner inFile1 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/pddl-parser-master/iplist.txt")).useDelimiter(",\\s*");

	    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
	    // List<String> temps = new LinkedList<String>();
	    List<String> temps = new ArrayList<String>();

	    // while loop
	    while (inFile1.hasNext()) {
	      // find next line
	      token1 = inFile1.next();
	      temps.add(token1);
	    }
	    inFile1.close();

	    String[] iplist = temps.toArray(new String[0]);
	    
	    System.out.println("iplist:");
	    for (String s : iplist) {
	      System.out.println(s);
	    }
	    
		
	 // create token1
	    String token2 = "";

	    // for-each loop for calculating heat index of May - October

	    // create Scanner inFile1
	    Scanner inFile2 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/pddl-parser-master/IdentifiedRoomList.txt")).useDelimiter(",\\s*");

	    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
	    // List<String> temps = new LinkedList<String>();
	    List<String> temps2 = new ArrayList<String>();

	    // while loop
	    while (inFile2.hasNext()) {
	      // find next line
	      token2 = inFile2.next();
	      temps2.add(token2);
	    }
	    inFile2.close();

	    String[] IdentifiedRoomList = temps2.toArray(new String[0]);
	    System.out.println("Identified Room List:");
	    for (String s : IdentifiedRoomList) {
	      System.out.println(s);
	    }
	    
	    
	 // create token1
	    String token3 = "";

	    // for-each loop for calculating heat index of May - October

	    // create Scanner inFile1
	    Scanner inFile3 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/pddl-parser-master/KeyPredicatesForInterdependence.txt")).useDelimiter(",\\s*");

	    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
	    // List<String> temps = new LinkedList<String>();
	    List<String> temps3 = new ArrayList<String>();

	    // while loop
	    while (inFile3.hasNext()) {
	      // find next line
	      token3 = inFile3.next();
	      temps3.add(token3);
	    }
	    inFile3.close();

	    String[] KeyPredicatesForInterdependence = temps3.toArray(new String[0]);
	    System.out.println("Key Predicates For Interdependence Based on capabilities (Action)");
	    for (String s : KeyPredicatesForInterdependence) {
	      System.out.println(s);
	    }
	    
	    // create token1
	    String token4 = "";

	    // for-each loop for calculating heat index of May - October

	    // create Scanner inFile1
	    Scanner inFile4 = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/pddl-parser-master/KnownActorTypeList.txt")).useDelimiter(",\\s*");

	    // Original answer used LinkedList, but probably preferable to use ArrayList in most cases
	    // List<String> temps = new LinkedList<String>();
	    List<String> temps4 = new ArrayList<String>();

	    // while loop
	    while (inFile4.hasNext()) {
	      // find next line
	      token4 = inFile4.next();
	      temps4.add(token4);
	    }
	    inFile4.close();

	    String[] KnownActorTypeList = temps4.toArray(new String[0]);
	    System.out.println("Known Goal Related Actor Type List:");
	    for (String s : KnownActorTypeList) {
	      System.out.println(s);
	    }
	    
		}
		catch(Exception e)
		{
			
		}
		
		
		
		
		
		/*String command = "python /Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/PDDL.py /Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/trydomain.pddl /Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/tryproblem.pddl";

        Process proc = Runtime.getRuntime().exec(command);

        // Read the output

        BufferedReader reader =  
              new BufferedReader(new InputStreamReader(proc.getInputStream()));

        String line = "";
        while((line = reader.readLine()) != null) {
            System.out.print(line + "\n");
        }*/

		
		// set up the command and parameter
		/*String pythonScriptPath = "/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/PDDL.py"; // /Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/trydomain.pddl /Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/tryproblem.pddl";
		String[] cmd = new String[4];
		cmd[0] = "python"; // check version of installed python: python -V
		cmd[1] = pythonScriptPath;
		cmd[2] = "/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/trydomain.pddl";
		cmd[3] = "/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/tryproblem.pddl";
		
		Runtime rt = Runtime.getRuntime();
		Process pr = rt.exec(cmd);
		
		// retrieve output from python script
		BufferedReader bfr = new BufferedReader(new InputStreamReader(pr.getInputStream()));
		String line = "";
		while((line = bfr.readLine()) != null) {
		// display each output line form python script
		System.out.println(line);
		}*/
        //proc.waitFor();
		
		/*String s = null;
		 
		try {
 
			// Process provides control of native processes started by ProcessBuilder.start and Runtime.exec.
			// getRuntime() returns the runtime object associated with the current Java application.
			Process p = Runtime.getRuntime().exec("python /Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/PDDL.py Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/trydomain.pddl /Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/tryproblem.pddl");
 
 
			//System.exit(0);
		} catch (IOException e) {
			System.out.println("exception happened - here's what I know: ");
			e.printStackTrace();
			System.exit(-1);
		}*/
		
		
		try {
			//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/counter.txt");
			//System.out.println(reader.read());
		/*
			File directory = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/MyStateFiles");

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
			}
			*/
			/*
			File directory2 = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/R1toR2");

			// Get all files in directory

			File[] files2 = directory2.listFiles();
			for (File file2 : files2)
			{
			   // Delete each file

			   if (!file2.delete())
			   {
			       // Failed to delete file

			       System.out.println("Failed to delete "+file2);
			   }
			}
			*/
			/*
			File directory2 = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/R1toR2");

			// Get all files in directory

			File[] files2 = directory.listFiles();
			for (File file2 : files2)
			{
			   // Delete each file

			   if (!file2.delete())
			   {
			       // Failed to delete file

			       System.out.println("Failed to delete "+file2);
			   }
			}*/
			
			int tex= 1000;
			FileWriter myCWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/counter.txt");
			myCWriter.write(new Integer(tex).toString());
		    myCWriter.close();
		    
		    
		    int tex2= 0;
			FileWriter myCWriter2 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/check.txt");
			myCWriter2.write(new Integer(tex2).toString());
		    myCWriter2.close();
		    
		    int tex3= 0;
			FileWriter myCWriter3 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/roomfadddatacheck.txt");
			myCWriter3.write(new Integer(tex3).toString());
		    myCWriter3.close();
		    
		    int tex4=0;
		    FileWriter myWriter3 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/shareddatausedcount.txt");
		    myWriter3.write(new Integer(tex4).toString());
		    myWriter3.close();
		    System.out.println("Successfully wrote to the file.");
		    
		    int tex6=0;
		    FileWriter myWriter5 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/shareddatausedcountrobotrelated.txt");
		    myWriter5.write(new Integer(tex6).toString());
		    myWriter5.close();
		    System.out.println("Successfully wrote to the file.");
		    
		    int tex5=0;
		    FileWriter myWriter4 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/FinalSolution/froomfadddatacheckExit.txt");
		    myWriter4.write(new Integer(tex5).toString());
		    myWriter4.close();
		    System.out.println("Successfully wrote to the file.");
		    
		    /*
		    FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/FinalSolution/finalstepitaration.txt");
		      myWriter.write("");
		      myWriter.close();
		      System.out.println("Successfully wrote to the file.");
		     */ 
		      FileWriter myWriter7 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/FinalSolution/finalstepitarationOnlyHimself.txt");
		      myWriter7.write("");
		      myWriter7.close();
		      System.out.println("Successfully wrote to the file.");
		      
		      FileWriter myWriter2 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/FinalSolution/finalstepitarationOnlySteps.txt");
		      myWriter2.write("");
		      myWriter2.close();
		      System.out.println("Successfully wrote to the file.");
		      
		      int t =0;
		      FileWriter myWriter8 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/dataexchangecount.txt");
		      myWriter8.write(new Integer(t).toString());
		      myWriter8.close();
		      System.out.println("Successfully wrote to the file.");
		      
		      int ts =0;
		      FileWriter myWriter9 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/TotalSleepTime.txt");
		      myWriter9.write(new Integer(ts).toString());
		      myWriter9.close();
		      System.out.println("Successfully wrote to the file.");
		      
		      int l =0;
		      FileWriter myWriter10 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/NumberofAllData.txt");
		      myWriter10.write(new Integer(l).toString());
		      myWriter10.close();
		      System.out.println("Successfully wrote to the file.");
		      
		      int l1 =0;
		      FileWriter myWriter101 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/ReadTimesData.txt");
		      myWriter101.write(new Integer(l1).toString());
		      myWriter101.close();
		      System.out.println("Successfully wrote to the file.");
		      
		      /*FileWriter myWriter6 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/StatePredicatesSent.txt");
		      myWriter6.write("");
		      myWriter6.close();
		      System.out.println("Successfully wrote to the file.");
		      
		      FileWriter myWriter7 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/R1toR2/ObjectSent.txt");
		      myWriter7.write("");
		      myWriter7.close();
		      System.out.println("Successfully wrote to the file.");*/
		      
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		try {
		    
		    int tex3= 0;
			FileWriter myCWriter3 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/roomfadddatacheckExit.txt");
			myCWriter3.write(new Integer(tex3).toString());
		    myCWriter3.close();
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		/*try {
			//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/counter.txt");
			//System.out.println(reader.read());
			
			Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/roomfadddatacheck.txt"));
			
			int t = 0;
			while(sc.hasNextInt())
			{
				roomfcounter  = sc.nextInt();
			}
			//System.out.println(tll[0]);
			
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}*/
		
		try {
			Thread.sleep(60);
		} catch (InterruptedException e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}
		Scanner scannerts = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/TotalSleepTime.txt"));
		
		int x1 = 0;
		while(scannerts.hasNextInt())
		{
	     x1 = scannerts.nextInt();
	     x1= x1+1;
		}
		scannerts.close();
		FileWriter myWriter81 = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/TotalSleepTime.txt");
		myWriter81.write(new Integer(x1).toString());
		myWriter81.close();
		System.out.println("Successfully wrote to the file.");
		
		
		String path="/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/MyStateFiles/";
		String fileprefix = "filename";
		String fileType =".pddl";
		
		mcall(args);
		//Main scriptPython = new Main();
        //scriptPython.runScript();
		
		
		/*String cmd = "python /Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/PDDL.py /Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/trydomain.pddl /Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/src/main/java/edu/kit/aquaplanning/pddl-parser-master/tryproblem.pddl";
		Runtime run = Runtime.getRuntime();
		Process pr = run.exec(cmd);
		pr.waitFor();
		BufferedReader buf = new BufferedReader(new InputStreamReader(pr.getInputStream()));
		String line = "";
		//System.out.println(buf.readLine());
		while ((line=buf.readLine())!=null) {
		System.out.println(line);
		}
		*/
		
		//ProcessBuilder pb = new ProcessBuilder("python"," -B /Users/vijayanthtummala/Downloads/pddl-parser-master/PDDL.py","/Users/vijayanthtummala/Downloads/pddl-parser-master/trydomain.pddl","/Users/vijayanthtummala/Downloads/pddl-parser-master/tryproblem.pddl");
		//Process p = pb.start();
		
		/*Process p = Runtime.getRuntime().exec("python -B /Users/vijayanthtummala/Downloads/pddl-parser-master/PDDL.py /Users/vijayanthtummala/Downloads/pddl-parser-master/trydomain.pddl /Users/vijayanthtummala/Downloads/pddl-parser-master/tryproblem.pddl");
		BufferedReader in = new BufferedReader(new InputStreamReader(p.getInputStream()));
		String ret = in.readLine();
		System.out.println("value is : "+ret);*/
		
		
				
		
		int i=0;
		boolean hasNext = true;
		
		//args[0]="/Users/vijayanthtummala/Downloads/trydomain.pddl";
		//args[1]="/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/MyStateFiles/filename0.pddl";
		int temp=0;
		while (hasNext)
		{
		    
		    	Scanner scannertss = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/filecount.txt"));
		
		int x1s = 0;
		while(scannertss.hasNextInt())
		{
	     x1s = scannertss.nextInt();
	     x1s= x1s+1;
		}
		scannertss.close();
		FileWriter myWriter81s = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/filecount.txt");
		myWriter81s.write(new Integer(x1s).toString());
		myWriter81s.close();
		System.out.println("Successfully wrote to the file.");
		
		
			String filePath = path+fileprefix+String.valueOf(i)+fileType;
			args[0]="/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/trydomain.pddl";
			args[1]=filePath;
			
			File f = new File(filePath);
			try {
			if(f.exists() && !f.isDirectory()) { 
				int roomfcounter=0;
				try {
					//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/counter.txt");
					//System.out.println(reader.read());
					
					Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/roomfadddatacheck.txt"));
					
					int t = 0;
					while(sc.hasNextInt())
					{
						roomfcounter  = sc.nextInt();
					}
					//System.out.println(tll[0]);
					//roomfcounter++;
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				

				
				
				
				try {
					//FileReader reader = new FileReader("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/counter.txt");
					//System.out.println(reader.read());
					
					Scanner sc = new Scanner(new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/observedexitcheck.txt"));
					
					//int t = 0;
					while(sc.hasNextInt())
					{
						 temp = sc.nextInt();
					}
					//System.out.println(tll[0]);
					//roomfcounter++;
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
				
		    				
			
				mcall(args);
				if (temp==0)
				{
				mcall(args);
				}
				else {
				try {
					
				      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/observedexitcheck.txt");
				      myWriter.write("0");
				      myWriter.close();
				      System.out.println("Successfully wrote to the file.");
				      //break;
				    } catch (IOException e) {
				      System.out.println("An error occurred.");
				      e.printStackTrace();
				    }
				}
				i++;
				if(i==15)
				{
				    try {
					
				      FileWriter myWriter = new FileWriter("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/value.txt");
				      myWriter.write("15");
				      myWriter.close();
				      System.out.println("Successfully wrote to the file.");
				      //break;
				    } catch (IOException e) {
				      System.out.println("An error occurred.");
				      e.printStackTrace();
				    }
				
				}
			}
			else {
				hasNext=false;
			}
			}
			catch (Exception e) {
				Logger.log(Logger.ERROR, "No More files to be read");
				e.printStackTrace();
			}
			
			//mcall(args);
		}
		
		/*File directory2 = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master/R1toR2");

		// Get all files in directory

		File[] files2 = directory2.listFiles();
		for (File file2 : files2)
		{
		   // Delete each file

		   if (!file2.delete())
		   {
		       // Failed to delete file

		       System.out.println("Failed to delete "+file2);
		   }
		}
		*/
		
		/* File dir = new File("/Users/vijayanthtummala/Downloads/aquaplanning-master-ro2/MyStateFiles");
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
