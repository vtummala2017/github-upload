#!/usr/bin/env python
# Four spaces as indentation [no tabs]

import re
from action import Action
import collections

class PDDL_Parser:

    SUPPORTED_REQUIREMENTS = [':strips', ':negative-preconditions', ':typing', ':adl']

    
    


    #-----------------------------------------------
    # Tokens
    #-----------------------------------------------

    def scan_tokens(self, filename):
        with open(filename,'r') as f:
            # Remove single line comments
            str = re.sub(r';.*$', '', f.read(), flags=re.MULTILINE).lower()
        # Tokenize
        stack = []
        list = []
        for t in re.findall(r'[()]|[^\s()]+', str):
            if t == '(':
                stack.append(list)
                list = []
            elif t == ')':
                if stack:
                    l = list
                    list = stack.pop()
                    list.append(l)
                else:
                    raise Exception('Missing open parentheses')
            else:
                list.append(t)
        if stack:
            raise Exception('Missing close parentheses')
        if len(list) != 1:
            raise Exception('Malformed expression')
        return list[0]

    #-----------------------------------------------
    # Parse domain
    #-----------------------------------------------

    def parse_domain(self, domain_filename):
        tokens = self.scan_tokens(domain_filename)
        if type(tokens) is list and tokens.pop(0) == 'define':
            self.domain_name = 'unknown'
            self.requirements = []
            self.types = {}
            self.objects = {}
            self.actions = []
            self.predicates = {}
            while tokens:
                group = tokens.pop(0)
                t = group.pop(0)
                if t == 'domain':
                    self.domain_name = group[0]
                elif t == ':requirements':
                    for req in group:
                        if not req in self.SUPPORTED_REQUIREMENTS:
                            raise Exception('Requirement ' + req + ' not supported')
                    self.requirements = group
                elif t == ':constants':
                    self.parse_objects(group, t)
                elif t == ':predicates':
                    self.parse_predicates(group)
                elif t == ':types':
                    self.parse_types(group)
                elif t == ':action':
                    self.parse_action(group)
                else: self.parse_domain_extended(t, group)
        else:
            raise Exception('File ' + domain_filename + ' does not match domain pattern')

    def parse_domain_extended(self, t, group):
        print(str(t) + ' is not recognized in domain')

    #-----------------------------------------------
    # Parse hierarchy
    #-----------------------------------------------

    def parse_hierarchy(self, group, structure, name, redefine):
        list = []
        while group:
            if redefine and group[0] in structure:
                raise Exception('Redefined supertype of ' + group[0])
            elif group[0] == '-':
                if not list:
                    raise Exception('Unexpected hyphen in ' + name)
                group.pop(0)
                type = group.pop(0)
                if not type in structure:
                    structure[type] = []
                structure[type] += list
                list = []
            else:
                list.append(group.pop(0))
        if list:
            if not 'object' in structure:
                structure['object'] = []
            structure['object'] += list

    #-----------------------------------------------
    # Parse objects
    #-----------------------------------------------

    def parse_objects(self, group, name):
        self.parse_hierarchy(group, self.objects, name, False)

    # -----------------------------------------------
    # Parse types
    # -----------------------------------------------

    def parse_types(self, group):
        self.parse_hierarchy(group, self.types, 'types', True)

    #-----------------------------------------------
    # Parse predicates
    #-----------------------------------------------

    def parse_predicates(self, group):
        for pred in group:
            predicate_name = pred.pop(0)
            if predicate_name in self.predicates:
                raise Exception('Predicate ' + predicate_name + ' redefined')
            arguments = {}
            untyped_variables = []
            while pred:
                t = pred.pop(0)
                if t == '-':
                    if not untyped_variables:
                        raise Exception('Unexpected hyphen in predicates')
                    type = pred.pop(0)
                    while untyped_variables:
                        arguments[untyped_variables.pop(0)] = type
                else:
                    untyped_variables.append(t)
            while untyped_variables:
                arguments[untyped_variables.pop(0)] = 'object'
            self.predicates[predicate_name] = arguments

    #-----------------------------------------------
    # Parse action
    #-----------------------------------------------

    def parse_action(self, group):
        name = group.pop(0)
        if not type(name) is str:
            raise Exception('Action without name definition')
        for act in self.actions:
            if act.name == name:
                raise Exception('Action ' + name + ' redefined')
        parameters = []
        positive_preconditions = []
        negative_preconditions = []
        add_effects = []
        del_effects = []
        extensions = None
        global allactions
        allactions = []
        while group:
            t = group.pop(0)
            if t == ':parameters':
                if not type(group) is list:
                    raise Exception('Error with ' + name + ' parameters')
                parameters = []
                untyped_parameters = []
                p = group.pop(0)
                while p:
                    t = p.pop(0)
                    if t == '-':
                        if not untyped_parameters:
                            raise Exception('Unexpected hyphen in ' + name + ' parameters')
                        ptype = p.pop(0)
                        while untyped_parameters:
                            parameters.append([untyped_parameters.pop(0), ptype])
                    else:
                        untyped_parameters.append(t)
                while untyped_parameters:
                    parameters.append([untyped_parameters.pop(0), 'object'])
            elif t == ':precondition':
                self.split_predicates(group.pop(0), positive_preconditions, negative_preconditions, name, ' preconditions')
            elif t == ':effect':
                self.split_predicates(group.pop(0), add_effects, del_effects, name, ' effects')
            else: extensions = self.parse_action_extended(t, group)
        self.actions.append(Action(name, parameters, positive_preconditions, negative_preconditions, add_effects, del_effects, extensions))

        #allactions.append(name,parameters,positive_preconditions, add_effects)
        
        
        #print(word_freq)
        #allactions={name: [positive_preconditions, negative_preconditions]}
        
        print("In-Function")
        for p in parameters:
            print(p)
        print("In-Function")
        print(add_effects)
    def parse_action_extended(self, t, group):
        print(str(t) + ' is not recognized in action')

    #-----------------------------------------------
    # Parse problem
    #-----------------------------------------------

    def parse_problem(self, problem_filename):
        #g_init = set()
        def frozenset_of_tuples(data):
            return frozenset([tuple(t) for t in data])
        tokens = self.scan_tokens(problem_filename)
        if type(tokens) is list and tokens.pop(0) == 'define':
            self.problem_name = 'unknown'
            self.state = frozenset()
            self.positive_goals = frozenset()
            self.negative_goals = frozenset()
            while tokens:
                group = tokens.pop(0)
                t = group.pop(0)
                if t == 'problem':
                    self.problem_name = group[0]
                elif t == ':domain':
                    if self.domain_name != group[0]:
                        raise Exception('Different domain specified in problem file')
                elif t == ':requirements':
                    pass # Ignore requirements in problem, parse them in the domain
                elif t == ':objects':
                    self.parse_objects(group, t)
                elif t == ':init':
                    self.state = frozenset_of_tuples(group)
                    #g_init.add(group)
                elif t == ':goal':
                    print(t)
                    for g in group:
                        print(g)
                    
                    
                    #goal = []
                    global goal
                    goal = []
                    for val in g:
                        if val == "[":
                            goal = []
                        else:
                            goal.append(val)
                    for g in goal:
                        print(g[2])
                    #positive_goals = []
                    #negative_goals = []
                    #self.split_predicates(group[0], positive_goals, negative_goals, '', 'goals')
                    #self.positive_goals = frozenset_of_tuples(positive_goals)
                    #self.negative_goals = frozenset_of_tuples(negative_goals)
                else: self.parse_problem_extended(t, group)
        else:
            raise Exception('File ' + problem_filename + ' does not match problem pattern')

    def parse_problem_extended(self, t, group):
        print(str(t) + ' is not recognized in problem')

    #-----------------------------------------------
    # Split predicates
    #-----------------------------------------------

    def split_predicates(self, group, positive, negative, name, part):
        if not type(group) is list:
            raise Exception('Error with ' + name + part)
        if group[0] == 'and':
            group.pop(0)
        else:
            group = [group]
        for predicate in group:
            if predicate[0] == 'not':
                if len(predicate) != 2:
                    raise Exception('Unexpected not in ' + name + part)
                negative.append(predicate[-1])
            else:
                positive.append(predicate)


    
#-----------------------------------------------
# Main
#-----------------------------------------------
if __name__ == '__main__':
    ext_conditions =[]
    actor_type_we_know = ["robot","block","room"]  #Real robot will be trained to identify by goals example CNN algorithms detecting BLOCK etc 
    Major_Actor = ["ROBOT"]                     # This is knowledge base of Robot about Himself
    def printSubsInDelimeters(string) :
     
        import os
        pprint.pprint(os.getcwd())
        # Stores the indices
        dels = [];
        for i in range(len(string)):
         
            # If opening delimeter
            # is encountered
            if (string[i] == '[') :
                dels.append(i);
 
            # If closing delimeter
            # is encountered
            elif (string[i] == ']' and len(dels) != 0) :
 
              # Extract the position
              # of opening delimeter
                pos = dels[-1];
                dels.pop();
 
                # Length of substring
                length = i - 1 - pos;
 
                # Extract the substring
                ans = string[pos + 1 : pos + 1 + length];
                ext_conditions.append(ans)
                #print(ans);
                
    import sys, pprint
    domain = sys.argv[1]
    problem = sys.argv[2]
    parser = PDDL_Parser()
    print('----------------------------')
    pprint.pprint(parser.scan_tokens(domain))
    print('----------------------------')
    pprint.pprint(parser.scan_tokens(problem))
    
    print('----------------------------')
    parser.parse_domain(domain)
    parser.parse_problem(problem)
    
    print('Domain name: ' + parser.domain_name)
    print("My Check")
    #print(parser.actions)
    counter =1
    #print(parser.actions[0][1])
    for act in parser.actions:
        print(counter)
        print(act)
        allactions.append(str(act))
        counter = counter +1
    counter=1   
    print('----------------------------')
    print('Problem name: ' + parser.problem_name)
    print('Objects: ' + str(parser.objects))
    print('State: ' + str(parser.state))
    print('Positive goals: ' + str(parser.positive_goals))
    print('Negative goals: ' + str(parser.negative_goals))
    

    
    
    
    #def merge(lst1, lst2):
    #    return [a + [b[1]] for (a, b) in zip(lst1, lst2)]
    '''def merge(lst1, lst2):
        dict1 = collections.defaultdict(list)
      
        for e in lst1 + lst2:
            dict1[e[0]].append(e[1])
        dictlist = list()
      
        for key, value in dict1.items():
            dictlist.append([key]+value)
      
        return dictlist'''
    #def merge(lst1, lst2):
    #    return [(sub + [lst2[i][-1]]) for i, sub in enumerate(lst1)]
    temp_f_goal=[]
    temp_ext_conditions = []
    temp_effect_moves =[]
    temp_acl=[]
    f_goal =[]
    c =0
    
    print(goal)
    for g in goal:
        c =0
        if "forall" in str(g):
            for a in g[2]:
                while c==0:
                    f_goal.append(a)
                    print(a)
                    c = c+1
    #temp_f_goal = f_goal
    #print("Yay Yay Yay Yay Yay Yay Yay Yay Yay Yay Yay Yay Yay")
    #print(temp_f_goal)
    
    counter =1
    actions =[]
    add_effects=[]
    pos_preconditions =[]
    for p in allactions:
        print(p)
        print()
        for item in p.split("\n"):
            if "action" in item:
                print (item.strip())
                actions.append(item.strip())
                #print(p)
            if "add_effects" in item:
                print (item.strip())
                add_effects.append(item.strip())
                #print(p)
            if "positive_preconditions" in item:
                print (item.strip())
                pos_preconditions.append(item.strip())

        counter = counter +1
    str_actions=[]
    for a in actions:
        print(str(a.replace("action: ", ""))) 
        str_actions.append(str(a.replace("action: ", "")))
    
    print("here")
    #print(merge(actions,add_effects))
    ac_add_ef =[]
    ac_add_pc =[]
    ct= 0
    high= len(actions)
    for ct in range(0,high):
        temp = str(str_actions[ct]) + " " + str(add_effects[ct])
        ac_add_ef.append(temp)
    print()
    #print(ac_add_ef)
    
    ct= 0
    high= len(actions)
    for ct in range(0,high):
        temp = str(str_actions[ct]) + " " + str(pos_preconditions[ct])
        ac_add_pc.append(temp)
    print()
    
    for f in ac_add_ef:
        print(f)
    print("letseeeeeeeeeeeeeeeeeeeeeee")
    print()
    for f in ac_add_pc:
        print(f)
    
    #for f in add_effects:
    #    print(f)
    my =0
    len_temp_f_goal = len(f_goal)
    while len_temp_f_goal !=0:
        my=my+1
        final_ip=[]
        print("meeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
        print()
        print()
        tc=0
        print()
        acl=[]
        for f in f_goal:
            tc=0
            #print("yooooo")
            ##print(f)
            for a in ac_add_ef:
                #print(a)
                if str(f) in str(a):
                    #final_ip.append(f)
                    #print(tc)
                    tc=tc+1
                    if str(a) not in acl:
                        acl.append(a)
                        #print(a)
        print()
        
        effect_moves = []
        #print(str_actions)
        #print("ACL")
        #print(acl)
        for s in str_actions:
            for a in acl:
                if str(s) in str(a):
                    if str(s) not in effect_moves:
                       # print(s)
                        effect_moves.append(s)
        
        
        #print(set(effect_moves))
        #print(effect_moves)
        
    # effect_moves has moves (pick up drop) in effect based on goal
    # f_goal has overall goals (goal state)
    # acl has moves and effect
        
        
        #import re as regex
        print()
        print()
        print(effect_moves)
        print(f_goal)
        print(acl)
        print()
        print()
        
        '''for a in ac_add_pc:
            print(a)
        for a in effect_moves:
            print(a)
        print()
        print(effect_moves)
        print("lord")
        print()'''
        effect_preconditions=[]
        for e in effect_moves:
            for a in ac_add_pc:
                if str(e) in str(a):
                    if str(a) not in effect_preconditions:
                        effect_preconditions.append(a)
                        #print("lord")
                        #print(a)
        
        
        for e in effect_preconditions:
            print(e)
        
        for e in effect_preconditions:
            printSubsInDelimeters(str(e))
        #ac_add_pc
        
        #for a in acl:
        #    printSubsInDelimeters(str(a));
        
        for e in effect_moves:
            if str(e) in ext_conditions:
                ext_conditions.remove(e)
            #e = ''.join(i for i in e if not i in effect_moves)
            #print(e)
            #ext_conditions.remove(str(e))
        for e in f_goal:
            if str(e) in ext_conditions:
                ext_conditions.remove(e)
        #print(ext_conditions)
        for e in ext_conditions:
            print(e)
        
        
            #a = a.replace("'", "")
        
        new_ext_cond =[]
        for item in ext_conditions:
            new_ext_cond.extend(item.split())
        #[words for segments in ext_conditions for words in segments.split(',')]
        
        '''print("start")
        for e in new_ext_cond:
            print(e) 
        print("end")'''
        
        ext_conditions.clear()
        for e in new_ext_cond:
            ext_conditions.append(e)
        
        cl = []
        for a in ext_conditions:
            a = a.replace("' ,", "'")
            cl.append(re.sub(r'[^a-zA-Z0-9-?]','',a)) #have only these values in list and clean other unnecessary characters
            #print(new_string)
        
        for c in cl:
            print(c)
        
        ext_conditions.clear()
        for e in cl:
            ext_conditions.append(e)
        #print("try")
        #print(res)
        
        #print(matches)  
        print()  
        print("start")
        for e in ext_conditions:
            print(e) 
        print("end") 
        print()  
        for e in actor_type_we_know:
            print(e)
            for x in ext_conditions:
                if str(e) == x:
                    ext_conditions.remove(x)
        
        '''for x in ext_conditions:
            if "/?" in x:
                ext_conditions.remove(x)
        for e in ext_conditions:
            print(e)'''
        for e in ext_conditions:
            if "?" in e:
                ext_conditions.remove(e)
                
        print()
        for i in ext_conditions:
            print(i)
        print()
        # Make own character set and pass 
        # this as argument in compile method
        regex = re.compile('[@_!#$%^&*()<>?/\|}{~:]')
     #--------------------------start Removing unwanted characters--------------#   
        # Pass the string in search 
        # method of regex object.   
        for e in ext_conditions: 
            if(regex.search(e) == None):
                #print(e)
                #print("String is accepted")
                continue
              
            else:
                #print(e)
                #print("String is NOT Accepted")
                ext_conditions.remove(e)
            
        for e in ext_conditions: 
            if(regex.search(e) == None):
                continue
              
            else:
                #print(e)
                #print("String is NOT Accepted")
                ext_conditions.remove(e)
        for e in ext_conditions: 
            if(regex.search(e) == None):
                continue
              
            else:
               # print(e)
               # print("String is NOT Accepted")
                ext_conditions.remove(e)
        for e in ext_conditions: 
            if(regex.search(e) == None):
                continue
              
            else:
                #print(e)
                #print("String is NOT Accepted")
                ext_conditions.remove(e)
     #--------------------------end Removing unwanted characters--------------#      
        #print(ext_conditions) 
        #temp =[]
        #temp.append([ x for x in  ext_conditions if "?" not in x ])
        #print(temp) 
        
        res = []
        for i in ext_conditions:
            if i not in res:
                res.append(i)
        #print(res)
        ext_conditions.clear()
        for e in res:
            ext_conditions.append(e) 
        
        print(ext_conditions)
        '''for e in ext_conditions:
            print(e)'''
        print("confused")
        for f in f_goal:
            temp_f_goal.append(f)
            print(f)
        
        for f in ext_conditions:
            temp_ext_conditions.append(f)
            
            print(f)
        
        for f in effect_moves:
            temp_effect_moves.append(f)
        
        for f in acl:
            temp_acl.append(f)
            #temp_f_goal.append(t)
        #temp_f_goal = f_goal
        f_goal.clear()
        for f in ext_conditions:
            if f not in temp_f_goal:
                f_goal.append(f)
        #f_goal = ext_conditions
        print("f_goal")
        print(f_goal)
        print("confused")
        for f in ext_conditions:
            print(f)
        len_temp_f_goal =len(f_goal)
        ext_conditions.clear()
        effect_moves.clear()
        acl.clear()
        '''for t in f_goal:
            temp_f_goal.append(t)
            print(len(f_goal))
        f_goal.clear()
        
        ext_conditions.clear()
        f_goal = ext_conditions
        len_temp_f_goal= len(f_goal)
        
        print(my)
        print(f_goal)
        print(temp_f_goal)'''
        print()
        print()
        print()
        print(temp_effect_moves)
        print(temp_f_goal)
        print(temp_ext_conditions)
        print(temp_acl)
        
        No_Duplicate_temp_effect_moves = []
        [No_Duplicate_temp_effect_moves.append(x) for x in temp_effect_moves if x not in No_Duplicate_temp_effect_moves]
        
        
        No_Duplicate_temp_f_goal = []
        [No_Duplicate_temp_f_goal.append(x) for x in temp_f_goal if x not in No_Duplicate_temp_f_goal]
        
        
        upper_actor_type_we_know=[]
        for i in range(len(actor_type_we_know)):
            upper_actor_type_we_know.append(actor_type_we_know[i].upper())
        print()
        print()
        print()
        print("Very Important Information")
        print("Interdependence points :" ,No_Duplicate_temp_effect_moves)
        print("Key predicates (apart from actors):" ,No_Duplicate_temp_f_goal)
        print("Goal related Actor types we know (not particular actors):" ,upper_actor_type_we_know)
        
        
        
        print()
        print()
        print()
        print("TRy STate Information")
        print('State: ' + str(parser.state))
        print("TRy STate Information")
        
        
        #initstate = str(parser.init)
        
        #sets=[parser.state]

        #print([list(x) for x in sets])
        
        initstate = list(parser.state)
        initroom =[]
        for i in initstate:
            if actor_type_we_know[2] in str(i) and actor_type_we_know[1] not in str(i) and actor_type_we_know[0] not in str(i):
                initroom.append(i)
                print(i)
        print(initroom)
        
                
        str_initroom = str(initroom)
        
        print(str_initroom)
        x = str_initroom.split(",")
        
        r_names=[]
        ct=0
        for i in x:
            if ct % 2 ==1:
                x = str(i).replace("'","")
                x = x.replace(" ","")
                x = x.replace(")","")
                x = x.replace("(","")
                x = x.replace("]","")
                x = x.replace("[","")
                r_names.append(x)
                print(i)
            ct = ct + 1
        
        
        
        
        
        print()
        print()
        print()
        print("Very Important Information")
        print("Interdependence points :" ,No_Duplicate_temp_effect_moves)
        print("Key predicates (apart from actors):" ,No_Duplicate_temp_f_goal)
        print("Goal related Actor types we know (not particular actors):" ,upper_actor_type_we_know)
        print("Identified rooms : ", r_names)
        
        
        file1 = open('/Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/iplist.txt', 'w+')
        #L = ["This is Delhi \n", "This is Paris \n", "This is London \n"]
        #s = "Hello\n"
  
        # Writing a string to file
        #
        for i in No_Duplicate_temp_effect_moves:
            #t = i, "\n"
            file1.write(i +"\n")
  
        # Writing multiple strings
        # at a time
        #file1.writelines(No_Duplicate_temp_effect_moves)
  
        # Closing file
        file1.close()
        
        
        
        file1 = open('/Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/KeyPredicatesForInterdependence.txt', 'w+')
        #L = ["This is Delhi \n", "This is Paris \n", "This is London \n"]
        #s = "Hello\n"
  
        # Writing a string to file
        #
        for i in No_Duplicate_temp_f_goal:
            #t = i, "\n"
            file1.write(i +"\n")
  
        # Writing multiple strings
        # at a time
        #file1.writelines(No_Duplicate_temp_effect_moves)
  
        # Closing file
        file1.close()
        
        
        file1 = open('/Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/KnownActorTypeList.txt', 'w+')
        #L = ["This is Delhi \n", "This is Paris \n", "This is London \n"]
        #s = "Hello\n"
  
        # Writing a string to file
        #
        for i in upper_actor_type_we_know:
            #t = i, "\n"
            file1.write(i +"\n")
  
        # Writing multiple strings
        # at a time
        #file1.writelines(No_Duplicate_temp_effect_moves)
  
        # Closing file
        file1.close()
        
        
        file1 = open('/Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/IdentifiedRoomList.txt', 'w+')
        #L = ["This is Delhi \n", "This is Paris \n", "This is London \n"]
        #s = "Hello\n"
  
        # Writing a string to file
        #
        for i in r_names:
            #t = i, "\n"
            file1.write(i +"\n")
  
        # Writing multiple strings
        # at a time
        #file1.writelines(No_Duplicate_temp_effect_moves)
  
        # Closing file
        file1.close()
        
        #for i in r_names:
            
        '''for i in initroom:
            x = i.split(",")
        
        for i in x:
            print(i)'''
        
        '''print()
        x = x.replace("'", "")
        x = x.replace("(", "")
        x = x.replace(")", "")
        x = x.replace("[", "")
        x = x.replace("]", "")
        x = str_initroom.split(",")
        
        for i in x:
            print(i)'''
        #if my==3:
        #    len_temp_f_goal =0
    # effect_moves has moves (pick up drop) in effect based on goal
    # f_goal has overall goals (goal state)
    # acl has moves and effect
    # ext_conditions (pre conditions from the moves in the first step from goal)
    # now use this pre conditions to find if we can have   
    
    # (for key predicates) Self Note: even if we add other unwanted elements like bat, ball etc, they will not 
        #be used in the second phase of linking (as we will only use data steps linking to one of the major actor)
        # which we will do in the planner based on the major actor list      