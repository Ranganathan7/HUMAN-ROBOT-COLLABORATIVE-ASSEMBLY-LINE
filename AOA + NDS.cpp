
#include <bits/stdc++.h>

using namespace std;

class AOA_ABC{
    
    //variables and parameters used in PSO
public:
    int no_of_tasks;
    int no_of_robots;
    int no_of_stations;
    int CT; //stores c0
    int T; //no of iterations to be performed
    int p; //no of objects in water
    vector<vector<int> > population; //stores the population 
    vector<vector<int> > new_population;
    vector<vector<int> > process_alternatives;
    vector<vector<int> > new_process_alternatives;
	vector<int> counter;  //for scout phase
    vector<double> density; //stores density
    vector<double> volume; //stores volume
    vector<double> acceleration; //stores acceleration
    vector<double> new_acceleration;
    double TF; //transfer operator
    double d; //density decreasing factor
    double max_acc;
    double min_acc;
    vector<vector<int> > obj_value; //holds the objective function values of each particle
    vector<vector<int> > input_data;
    vector<double> energy_consumption;
    vector<vector<vector<int> > > seq_dependent_time;
    vector<vector<int> > pre_matrix;
    vector<vector<int> > setup_time;
    unordered_map<int, unordered_set<int> > pre_graph;
    vector<vector<int> > EA_seq;
    vector<vector<int> > EA_pa;
    vector<vector<int> > EA_obj_values;
    unordered_map<int, vector<int> > F_values;
    vector<int> g_best;
    vector<int> g_best_process_alternative;
    double g_best_density;
    double g_best_volume;
    double g_best_acceleration;
    
    //getting input from user
    void get_input(string s){
    //gets the input from user and generates a precedence graph
        
        //cout<<"Enter number of stations : ";
        
        
        ifstream file(s);
		file.seekg(0);
       file>>no_of_tasks;
       file>>no_of_robots;
       vector<vector<int> > temp_input_data(2*no_of_robots + 1, vector<int>(no_of_tasks));
       vector<vector<vector<int> > > temp_seq_dependent_time(no_of_robots, vector<vector<int>>(no_of_tasks, vector<int>(no_of_tasks)));
       vector<vector<int> > temp_setup_time(no_of_robots, vector<int>(no_of_tasks));
       vector<vector<int> > temp_pre_matrix(no_of_tasks, vector<int>(no_of_tasks));
       vector<double> temp_energy_consumption(no_of_robots+1);
       input_data = temp_input_data;
       seq_dependent_time = temp_seq_dependent_time;
       setup_time = temp_setup_time;
       pre_matrix = temp_pre_matrix;
       energy_consumption = temp_energy_consumption;
       //cout<<endl<<"OPERATION TIMES FOR HRC\n";
        for(int i=0;i<(no_of_robots*2 + 1); i++){
            for(int j=0;j<no_of_tasks;j++){
                file>>input_data[i][j];
            }
        }
       //cout<<endl<<"SETUP TIMES FOR ROBOTS\n";
        for(int i=0;i<no_of_robots; i++){
            for(int j=0;j<no_of_tasks;j++){
                file>>setup_time[i][j];
            }
        }
        for(int i=0;i<no_of_robots;i++){
           //cout<<endl<<"SEQUENCE DEPENDENT TIME FOR ROBOT "<<i+1<<endl;
           for(int j=0;j<no_of_tasks;j++){
               for(int k=0;k<no_of_tasks;k++){
                   file>>seq_dependent_time[i][j][k];
               }
           }
           //cout<<endl;
       }
       //cout<<endl<<"PRECEDENCE MATRIX"<<endl;
       for(int i=0;i<no_of_tasks;i++){
           for(int j=0;j<no_of_tasks;j++){
            	file>>pre_matrix[i][j];
    		}
       }
       //cout<<endl<<"ENERGY CONSUMPTION"<<endl;
       for(int i=0;i<(no_of_robots+1);i++){
           file>>energy_consumption[i];
       }
       file.close();
    	no_of_stations = no_of_robots;
    	calculate_initial_cycle_time();
      
        /*cout<<endl<<"NUMBER OF TASKS : "<<no_of_tasks;
        cout<<endl<<"NUMBER OF ROBOTS : "<<no_of_robots;
        cout<<endl<<"NUMBER OF STATIONS : "<<no_of_stations;
        cout<<endl<<"NUMBER OF ITERATIONS : "<<T;
        cout<<endl<<"SIZE OF THE POPULATION : "<<p;
        cout<<endl;
        //printing
        cout<<endl<<"OPERATION TIMES FOR HRC\n";
        for(int i=0;i<(2*no_of_robots + 1); i++){
            for(int j=0;j<no_of_tasks;j++){
                cout<<input_data[i][j]<<" ";
            }
            cout<<endl;
        }
        //printing
        cout<<endl<<"SETUP TIMES FOR ROBOTS\n";
        for(int i=0;i<no_of_robots; i++){
            for(int j=0;j<no_of_tasks;j++){
                cout<<setup_time[i][j]<<" ";
            }
            cout<<endl;
        }
        //printing
       for(int i=0;i<no_of_robots;i++){
           cout<<endl<<"SEQUENCE DEPENDENT TIME FOR ROBOT "<<i+1<<endl;
           for(int j=0;j<no_of_tasks;j++){
               for(int k=0;k<no_of_tasks;k++){
                   cout<<seq_dependent_time[i][j][k]<<" ";
               }
               cout<<endl;
           }
           cout<<endl;
       }
        //printing
       cout<<endl<<"PRECEDENCE MATRIX"<<endl;
       for(int i=0;i<no_of_tasks;i++){
           for(int j=0;j<no_of_tasks;j++){
            cout<<pre_matrix[i][j]<<" ";
           }
           cout<<endl;
       }
        //printing
       cout<<endl<<"ENERGY CONSUMPTION"<<endl;
       for(int i=0;i<(no_of_robots+1);i++){
           cout<<energy_consumption[i]<<" ";
       }
       cout<<endl;
       */
    
        for(int i=1;i<=no_of_tasks;i++){
            //cout<<"ENTER PRECEDENCE TASKS FOR TASK "<<i<<": (enter 0 if no precedence):\n";
            for(int j=0;j<no_of_tasks;j++){
                if(pre_matrix[j][i-1]==0) continue;
                pre_graph[i].insert(j+1);
            }
        }

    }
    
    
    vector<int> generate_process_alternative(){
        //geenrates process alternative for each station 
        // 1 = robot
        // 2 = cobot
        // 3 = human
        vector<int> ans;
        for(int i=0;i<no_of_stations;i++){
            if(i==0){
                int random = 1 + rand()%3;
                ans.push_back(random);
            }
            else{
                if(ans[i-1]==1){
                    int random = 2 + rand()%2;
                    ans.push_back(random);
                }
                else if(ans[i-1]==2){
                    int random = 1 + rand()%2;
                    if(random==2){
                        ans.push_back(random+1);
                    }
                    else ans.push_back(random);
                }
                else{
                    int random = 1 + rand()%2;
                    ans.push_back(random);
                }
            }
        }
        return ans;
    }
    
    //function that generates a single sequnce according to the given precedence relations
    vector<int> generate_sequence_randomly(unordered_map<int, unordered_set<int> > pre){
    
        int n = 0; //number of tasks in the sequence(assigned)
        vector<int> seq;  //holds the sequence generated
        unordered_set<int> s; //available set that holds tasks that can be assigned
        unordered_set<int> assigned; //holds the already assigned tasks

        //creating the available set
        for(int i=1;i<=no_of_tasks;i++){
            if(pre[i].size()==0){
                s.insert(i);
            }
        }
        while(n < no_of_tasks){
            n = n + 1;
            int x = s.size();
            int random = rand()%x;
            unordered_set<int>::iterator it = s.begin();
            advance(it, random);
            seq.push_back(*it);
            assigned.insert(*it);
            //removing the assigned task from available set and adding the tasks that had this task as its precedence in the available set
            int task = *it;
            s.erase(it);
        
            pre.erase(task);
            for(int i=1;i<=no_of_tasks;i++){
                if(assigned.find(i)==assigned.end()){
                    if(pre[i].find(task)!=pre[i].end()) pre[i].erase(task);
                    if(pre[i].size()==0) s.insert(i);
                }
            }
        
        }

        return seq;
    }
    
    
    //generates initial density, volume, acceleration
    double generate_random_number(){
        double random = rand()%100;
        random = random / 100;
        return random;
    }
    
    //generates multiple sequences according to the population size
    void generate_population(){
        for(int i=0;i<p;i++){
            population.push_back(generate_sequence_randomly(pre_graph));
            new_population.push_back(population[i]);
            process_alternatives.push_back(generate_process_alternative());
            new_process_alternatives.push_back(process_alternatives[i]);
            density.push_back(generate_random_number());
            volume.push_back(generate_random_number());
            acceleration.push_back(generate_random_number());
            new_acceleration.push_back(acceleration[i]);
        }   
		for(int i=0;i<p;i++){
		    int ct = objective_function(population[i], process_alternatives[i]);
		    int energy = energy_calculation(population[i], process_alternatives[i], ct);
		    vector<int> temp;
		    temp.push_back(ct);
		    temp.push_back(energy);
            obj_value.push_back(temp);
            counter.push_back(0);
        } 
    }
    
    
    
    void calculate_initial_cycle_time(){
    //(i,j) -> i defines the robot number and j defines the activity number (input_data)
    
    //tasks = given sequence of task allocation by the optimization algorithm
    
        CT = 0;  
        int no_robots = no_of_robots; //no of rows in the input_data matrix 
        int no_tasks = no_of_tasks;  //no of cols in the input_data matrix
    
    //for getting the min time possible for that allocation (using the formula given in slides)
        for(int j=0;j<no_tasks;j++){
            int min_time = INT_MAX;   //min time taken by the robots to complete the ith task
            for(int i=0;i<(2*no_robots + 1);i++){
                min_time = min(min_time, input_data[i][j]);
            }
            CT+=min_time;
        }
        CT = CT / no_of_stations;
	}
    //objective function calculation (cycle time)
    int objective_function(vector<int> tasks, vector<int> process_alternative){
    
    	int cycle_time = CT;
        int no_robots = no_of_robots; //no of rows in the input_data matrix 
        int no_tasks = no_of_tasks;  //no of cols in the input_data matrix
        //cout<<endl<<"INITIAL CYCLE TIME : "<<cycle_time<<endl;
        while(true){
        //assigning the start task for the robots
            int start = 0;
            int si;
        //iterating through each station for given cycle time
            for(int s=0;s<no_of_stations;s++){
                si = start;  //si = start index
        
            //if all tasks assigned, the while loop ends
                if(si>=no_tasks) break;
            
                int n = 0; //most no of tasks assigned to a process alternative in that station
                
                if(process_alternative[s]==1){
                //looking for best robot
                for(int  r = 0; r<no_robots; r++){
                    int t = 0; //respective time taken by that robot in that station
                    si = start;
                    while(t<=cycle_time){
                        if(si>=no_tasks) break;
                        //task constraint
                        if(input_data[r][tasks[si]-1]<=0){
                        	si = start;
                        	break;
						}
                        t+=input_data[r][tasks[si]-1];  
                        //adding the sequence dependent times and setup times of the robot
                        t+=setup_time[r][tasks[si]-1];
                        if(si!=start){
                            t+=seq_dependent_time[r][tasks[si]-1][tasks[si-1]-1];
                        }
                        if(t>cycle_time) break;
                        si++;
                    }
            
                    n = max(n, si-start); //(si-start)= no of tasks assigned to that particular robot
                }
                }
                
                else if(process_alternative[s]==2){
                //looking for best cobot
                for(int  r = no_robots; r<(2*no_robots); r++){
                    int t = 0; //respective time taken by that cobot in that station
                    si = start;
                    while(t<=cycle_time){
                        if(si>=no_tasks) break;
                        t+=input_data[r][tasks[si]-1];  
                        //adding the sequence dependent times and setup times of the robot
                        t+=setup_time[r-no_of_robots][tasks[si]-1];
                        if(si!=start){
                            t+=seq_dependent_time[r-no_of_robots][tasks[si]-1][tasks[si-1]-1];
                        }
                        if(t>cycle_time) break;
                        si++;
                    }
            
                    n = max(n, si-start); //(si-start)= no of tasks assigned to that particular robot
                }
                }
                
                else{
                //for human
                for(int  r = 2*no_robots; r<(2*no_robots + 1); r++){
                    int t = 0; //respective time taken by that robot in that station
                    si = start;
                    while(t<=cycle_time){
                        if(si>=no_tasks) break;
                        t+=input_data[r][tasks[si]-1];  
                        //adding the sequence dependent times and setup times of the robot
                        if(t>cycle_time) break;
                        si++;
                    }
            
                    n = max(n, si-start); //(si-start)= no of tasks assigned to that particular robot
                }
                }
            
                start+=n;
            }
        
            if(start>=no_tasks){
                break;
            }
            else cycle_time++;
        }
    
        return cycle_time;
    }
    
    
    
    
    
    //prints the current iteration values
    void print_iteration(int t){
        cout<<endl<<"\n\nITERATION "<<t<<" :\n"<<endl;
        cout<<"population = objective function value(cycle_time, energy) :\n";
        for(int i=0;i<p;i++){
            for(int j=0;j<no_of_tasks;j++){
                cout<<population[i][j]<<" ";
            }
            cout<<"  =  "<<obj_value[i][0]<<" , "<<obj_value[i][1];
            cout<<endl;
        }
        cout<<"process alternative for each station :\n";
        for(int i=0;i<p;i++){
            for(int j=0;j<no_of_stations;j++){
                cout<<process_alternatives[i][j]<<" ";
            }
            cout<<endl;
        }
        cout<<endl;
        /*cout<<"\ndensity, volume, acceleration of each object :\n";
        for(int i=0;i<p;i++){
            cout<<density[i]<<", "<<volume[i]<<", "<<acceleration<<[i]<<endl;
        }
        cout<<endl;*/
    }

    
    
    //for updating the velocity and position in each iteration
    vector<int> subtract_position_position(vector<int> x1, vector<int> x2){
        vector<int> ans;
        for(int i=0;i<x1.size();i++){
            if(x1[i]==x2[i]) ans.push_back(0);
            else{
                ans.push_back(x1[i]);
            }
        }
        return ans;
    }



    vector<int> multiply_coeff(double c, vector<int> v){
        vector<int> ans;
        for(int i=0;i<v.size();i++){
            double random = rand()%100;
            random = random / 100;
            if(random<c){
                ans.push_back(v[i]);
            }
            else{
                ans.push_back(0);
            }
        }
        return ans;
    }


    vector<int> add_position_position(vector<int> v1, vector<int> v2){
        vector<int> ans;
        for(int i=0;i<v1.size();i++){
            if(v1[i]!=0 && v2[i]==0) ans.push_back(v1[i]);
            else ans.push_back(v2[i]);
        }
        return ans;
    }
    

    
    //updates the population
    void update_population(int t){
        NDS();
        //update density and volume
        update_density_volume();
        //update TF and d
        update_TF_d(t);
        //update acceleration with normalisation
        update_acceleration();
        //update task sequence and process alternative
        change_object();
        
    }
    
    //updates density and volume
    void update_density_volume(){
        for(int i=0;i<p;i++){
            double random = rand()%100;
            random = random / 100;
            density[i] = density[i] + (random*(g_best_density - density[i]));
            volume[i] = volume[i] + (random*(g_best_volume - volume[i]));
        }
    }
    
    //updates TF and d
    void update_TF_d(int t){
        TF = exp((t-T)/T);
        d = (exp((T-t)/T) - (t/T));
    }
    
    //update acceleration and normalises it
    void update_acceleration(){
        max_acc = INT_MIN;
        min_acc = INT_MAX;
        if(TF<=0.5){
            for(int i=0;i<p;i++){
                int partner = rand()%p;
                new_acceleration[i] = ((density[partner] + volume[partner]*acceleration[partner])/(density[i]*volume[i]));
                max_acc = max(max_acc, new_acceleration[i]);
                min_acc = min(min_acc, new_acceleration[i]);
            }
        }
        else{
            for(int i=0;i<p;i++){
                new_acceleration[i] = ((g_best_density + g_best_volume*g_best_acceleration)/(density[i]*volume[i]));
                max_acc = max(max_acc, new_acceleration[i]);
                min_acc = min(min_acc, new_acceleration[i]);
            }
        }
        acceleration = new_acceleration;
        //normalising
        for(int i=0;i<p;i++){
            acceleration[i] = 0.9 * ((acceleration[i] - min_acc)/(max_acc - min_acc));
            acceleration[i] = acceleration[i] + 0.1;
        }
    }
    
    //update objects
    void change_object(){
        
        if(TF<=0.5){
            for(int i=0;i<p;i++){
                int partner = rand()%p;
                new_population[i] = change_task_sequence(i, partner);
                new_process_alternatives[i] = change_process_alternative(i, partner);
                if(population[i]==new_population[i] && process_alternatives[i]==new_process_alternatives[i]){
                	counter[i]++;
				}
				else{
					counter[i] = 0;
					obj_value[i][0] = objective_function(new_population[i], new_process_alternatives[i]);
            		obj_value[i][1] = energy_calculation(new_population[i], new_process_alternatives[i], obj_value[i][0]);
				}
            }
        }
        else{
            for(int i=0;i<p;i++){
                new_population[i] = change_task_sequence(i);
                new_process_alternatives[i] = change_process_alternative(i);  
                if(population[i]==new_population[i] && process_alternatives[i]==new_process_alternatives[i]){
                	counter[i]++;
				}
				else{
					counter[i] = 0;
					obj_value[i][0] = objective_function(new_population[i], new_process_alternatives[i]);
            		obj_value[i][1] = energy_calculation(new_population[i], new_process_alternatives[i], obj_value[i][0]);
				}
            }
        }
        population = new_population;
        process_alternatives = new_process_alternatives;
    }
    
    //to change task sequence with parameter
    vector<int> change_task_sequence(int index, int partner){
        vector<int> ans;
        double c = d * acceleration[index];
        ans = subtract_position_position(population[partner], population[index]);
        ans = multiply_coeff(c, ans);
        ans = add_position_position(population[index], ans);
        ans = normalise(ans, pre_graph);
        return ans;
    }
    
    //to change process alternative with paramter
    vector<int> change_process_alternative(int index, int partner){
        vector<int> ans;
        double c = d * acceleration[index];
        ans = subtract_position_position(process_alternatives[partner], process_alternatives[index]);
        ans = multiply_coeff(c, ans);
        ans = add_position_position(process_alternatives[index], ans);
        ans = normalise_process_alternative(ans);
        return ans;
    }
    
    //to change task sequence 
    vector<int> change_task_sequence(int index){
        vector<int> ans;
        double c = d * acceleration[index];
        ans = subtract_position_position(g_best, population[index]);
        ans = multiply_coeff(c, ans);
        int random = rand()%2;
        if(random==0){
            ans = add_position_position(population[index], ans);
        }
        else{
            ans = subtract_position_position(population[index], ans);
        }
        ans = normalise(ans, pre_graph);
        return ans;
    }
    
    //to change process alternative
    vector<int> change_process_alternative(int index){
        vector<int> ans;
        double c = d * acceleration[index];
        ans = subtract_position_position(g_best_process_alternative, process_alternatives[index]);
        ans = multiply_coeff(c, ans);
        int random = rand()%2;
        if(random==0){
            ans = add_position_position(process_alternatives[index], ans);
        }
        else{
            ans = subtract_position_position(process_alternatives[index], ans);
        }
        ans = normalise_process_alternative(ans);
        return ans;
    }
    
    
    //to normalise the updated population according to precedence relation
    vector<int> normalise(vector<int> pos, unordered_map<int, unordered_set<int> > pre){
        int n = 0;
        vector<int> seq; 
        unordered_set<int> s; 
        unordered_set<int> assigned; 

   
        for(int i=1;i<=no_of_tasks;i++){
            if(pre[i].size()==0){
                s.insert(i);
            }
        }
        while(n < no_of_tasks && seq.size() < no_of_tasks){
            int task;
            if(s.find(pos[n])!=s.end() && pos[n]!=0){
                seq.push_back(pos[n]);
                assigned.insert(pos[n]);
                task = pos[n];
                s.erase(pos[n]);
                n = n + 1;
            }
            else{
                int x = s.size();
                int random = pos[n]%x;
                unordered_set<int>::iterator it = s.begin();
                advance(it, random);
                seq.push_back(*it);
                assigned.insert(*it);
                task = *it;
                s.erase(it);
            }
        
            pre.erase(task);
            for(int i=1;i<=no_of_tasks;i++){
                if(assigned.find(i)==assigned.end()){
                    if(pre[i].find(task)!=pre[i].end()) pre[i].erase(task);
                    if(pre[i].size()==0) s.insert(i);
                }
            }
        }
        return seq;
    }
    
    vector<int> normalise_process_alternative(vector<int> process_alternative){
    	vector<int> ans;
    	if(process_alternative[0]<=0 || process_alternative[0]>3){
    		int random = 1 + rand()%3;
    		ans.push_back(random);
		}
		else{
			ans.push_back(process_alternative[0]);
		}
    	for(int i=1;i<no_of_stations;i++){
    		if(ans[i-1]==process_alternative[i]){
    			int random = 1 + rand()%3;
    			while(random==ans[i-1]){
    				random = 1 + rand()%3;
				}
				ans.push_back(random);
			}
			else if(process_alternative[i]<=0 || process_alternative[i]>3){
				int random = 1 + rand()%3;
				ans.push_back(random);
			}
			else{
				ans.push_back(process_alternative[i]);
			}
		}
		return  ans;
	}
    
    void print_solution(int ct, int energy, vector<int> tasks, vector<int> process_alternative){
        
        cout<<endl;
        //assigning the start task for the robots
        
        cout<<endl;
        //cout<<"BEST SOLUTION FOUND"<<endl;
        for(int i=0;i<no_of_tasks;i++){
            cout<<tasks[i]<<" ";
        }
        cout<<"= "<<ct<<" , "<<energy<<endl;
        for(int i=0;i<no_of_stations;i++){
        	cout<<process_alternative[i]<<" ";
		}
		cout<<endl;
        //assigning the start task for the robots
        int start = 0;
        int si;
        //iterating through each station for given cycle time
        for(int s=0;s<no_of_stations;s++){
            si = start;  //si = start index
        
            //if all tasks assigned, the while loop ends
            if(si>=no_of_tasks) break;
            int robo_index;
            int n = 0; //most no of tasks assigned to a robot in that station
            
            if(process_alternative[s]==1){
            for(int  r = 0; r<no_of_robots; r++){
                int t = 0; //respective time taken by that robot in that station
                si = start;
                while(t<=ct){
                    if(si>=no_of_tasks) break;
                    //task constraint
                    if(input_data[r][tasks[si]-1]<=0){
                        si = start;
                        break;
					}
                    t+=input_data[r][tasks[si]-1];  
                    //adding the sequence dependent times and setup times of the robot
                    t+=setup_time[r][tasks[si]-1];
                    if(si!=start){
                        t+=seq_dependent_time[r][tasks[si]-1][tasks[si-1]-1];
                    }
                    if(t>ct) break;
                    si++;
                }
                if((si-start) > n) {
                    robo_index = r;
                    n = si-start;
                }
                
            }
        	}
        	
            else if(process_alternative[s]==2){
            for(int  r = no_of_robots; r<2*no_of_robots; r++){
                int t = 0; //respective time taken by that robot in that station
                si = start;
                while(t<=ct){
                    if(si>=no_of_tasks) break;
                    t+=input_data[r][tasks[si]-1];  
                    //adding the sequence dependent times and setup times of the robot
                    t+=setup_time[r-no_of_robots][tasks[si]-1];
                    if(si!=start){
                        t+=seq_dependent_time[r-no_of_robots][tasks[si]-1][tasks[si-1]-1];
                    }
                    if(t>ct) break;
                    si++;
                }
                if((si-start) > n) {
                    robo_index = r;
                    n = si-start;
                }
                
            }
        	}
        	
            else{
            for(int  r = 2*no_of_robots; r<(2*no_of_robots + 1); r++){
                int t = 0; //respective time taken by that robot in that station
                si = start;
                while(t<=ct){
                    if(si>=no_of_tasks) break;
                    t+=input_data[r][tasks[si]-1];  

                    if(t>ct) break;
                    si++;
                }
                if((si-start) > n) {
                    robo_index = r;
                    n = si-start;
                }
                
            }
            }
            
            cout<<"\nstation number "<<s+1<<"'s allocation: "<<endl;
            if(process_alternative[s]==1){
            cout<<"robot assigned : "<<robo_index+1<<endl;
            }
            else if(process_alternative[s]==2){
            int cobot = robo_index - no_of_robots;
            cout<<"cobot assigned : "<<cobot+1<<endl;
            }
            else{
            cout<<"human assigned"<<endl;
            }
            cout<<"tasks assigned : ";
            int st = 0;
            for(int i=start;i<(start+n);i++){
                cout<<tasks[i]<<" ";
                st+=input_data[robo_index][tasks[i]-1];
                //adding the sequence dependent times and setup times of the robot
                if(process_alternative[s]==2){
                	st+=setup_time[robo_index-no_of_robots][tasks[i]-1];
				}
				if(process_alternative[s]==1){
                	st+=setup_time[robo_index][tasks[i]-1];
            	}
                if(i!=start){
                	int r_index = robo_index;
                	if(process_alternative[s]==2) r_index = r_index - no_of_robots;
                    if(process_alternative[s]==2 || process_alternative[s]==1)st+=seq_dependent_time[r_index][tasks[i]-1][tasks[i-1]-1];
                }
            }
            cout<<endl;
            cout<<"its cycle time : "<<st;
            cout<<endl;
            start+=n;
        }
    }
    
    //calculates energy consumption 
    int energy_calculation(vector<int> tasks, vector<int> process_alternative, int ct){
    	int si;
    	int start = 0;
    	double energy = 0; //total energy consumption in all the stations
    
    	for(int s=0;s<no_of_stations;s++){
        	si = start;  //si = start index
        
            //if all tasks assigned, the while loop ends
        	if(si>=no_of_tasks) break;
        	int robo_index;
        	int n = 0; //most no of tasks assigned to a robot in that station
            
    
            
        	if(process_alternative[s]==1){
        		for(int  r = 0; r<no_of_robots; r++){
            		int t = 0; //respective time taken by that robot in that station
            		si = start;
            		while(t<=ct){
                		if(si>=no_of_tasks) break;
                		//task constraint
                        if(input_data[r][tasks[si]-1]<=0){
                        	si = start;
                        	break;
						}
                		t+=input_data[r][tasks[si]-1];  
                    //adding the sequence dependent times and setup times of the robot
                		t+=setup_time[r][tasks[si]-1];
                		if(si!=start){
                    		t+=seq_dependent_time[r][tasks[si]-1][tasks[si-1]-1];
                		}
                		if(t>ct) break;
                		si++;
            		}
            		if((si-start) > n) {
                		robo_index = r;
                		n = si-start;
            		}
                
        		}
        	}
        	
        	else if(process_alternative[s]==2){
        		for(int  r = no_of_robots; r<2*no_of_robots; r++){
            		int t = 0; //respective time taken by that robot in that station
            		si = start;
            		while(t<=ct){
                		if(si>=no_of_tasks) break;
                		t+=input_data[r][tasks[si]-1];  
                    //adding the sequence dependent times and setup times of the robot
                		t+=setup_time[r-no_of_robots][tasks[si]-1];
                		if(si!=start){
                    		t+=seq_dependent_time[r-no_of_robots][tasks[si]-1][tasks[si-1]-1];
                		}
                		if(t>ct) break;
                		si++;
            		}
            		if((si-start) > n) {
                		robo_index = r - no_of_robots;
                		n = si-start;
            		}
                
        		}
        	}
        	
        	else{
            	int r = 2*no_of_robots;
            	int t = 0; //respective time taken by that robot in that station
            	si = start;
            	while(t<=ct){
                	if(si>=no_of_tasks) break;
                	t+=input_data[r][tasks[si]-1];  

                	if(t>ct) break;
                	si++;
            	}
            	robo_index = no_of_robots;
            	n = si - start;      
            
        	}
            
    
        	double st = 0; //energy consumed in s'th station
        	for(int i=start;i<(start+n);i++){
            	if(process_alternative[s]==2){
                	st = st + energy_consumption[robo_index]*(double(input_data[2*no_of_robots][tasks[i]-1]));
                	st = st + energy_consumption[robo_index]*(double(input_data[robo_index+no_of_robots][tasks[i]-1]));
            	}
            	else if(process_alternative[s]==3){
                	st = st + energy_consumption[robo_index]*(double(input_data[2*no_of_robots][tasks[i]-1]));
            	}	
            	else{
                	st = st + energy_consumption[robo_index]*(double(input_data[robo_index][tasks[i]-1]));
                
            	}
                
        	}
        	energy+=st;
        	start+=n;
    	}
    	int return_value = ceil(energy);
    	return return_value;
	}
	
	void NDS(){
	  
	    F_values.clear();
	    //finding only one front as front 1 has the best solution
	    for(int i=0;i<p;i++){
	        int n = 0;
	        for(int j=0;j<p;j++){
	            if(i==j) continue;
	           
	            if(is_dominating(obj_value[j], obj_value[i])){
	                n = n + 1;
	            }
	        }
	        if(n==0){
	            vector<int> temp;
	            temp.push_back(obj_value[i][0]);
	            temp.push_back(obj_value[i][1]);
	            
	            F_values[i] = temp;
	        }
	    }
	    update_EA();
	    crowding_distance(); //to find best solution in front1 and initialize g_best
	}
	
	bool is_dominating(vector<int> x, vector<int> y){
	    bool flag = false;
	    if(x[0]<y[0] && x[1]<y[1]){
	        flag = true;
	        return flag;
	    }
	    else return flag;
	}
	
	
	void crowding_distance(){
	    unordered_map<int, int> d; //stores the crowding distances value
	    int ct_max = INT_MIN, ct_min = INT_MAX, e_max = INT_MIN, e_min = INT_MAX;
	    int ct_max_index, ct_min_index, e_max_index, e_min_index;
	    for(auto x: F_values){
	        d.insert({x.first,0});
	        if(x.second[0]>ct_max){
	            ct_max = x.second[0];
	            ct_max_index = x.first;
	        }
	        if(x.second[0]<ct_min){
	            ct_min = x.second[0];
	            ct_min_index = x.first;
	        }
	        if(x.second[1]>e_max){
	            e_max = x.second[1];
	            e_max_index = x.first;
	        }
	        if(x.second[1]<e_min){
	            e_min = x.second[1];
	            e_min_index = x.first;
	        }
	    }
	    d[ct_max_index] = INT_MAX;
	    d[ct_min_index] = INT_MAX;
	    d[e_max_index] = INT_MAX;
	    d[e_min_index] = INT_MAX;
	    for(auto x:d){
	        if(x.second==INT_MAX){
	            g_best = population[x.first];
	            g_best_process_alternative = process_alternatives[x.first];
	            g_best_density = density[x.first];
	            g_best_volume = volume[x.first];
	            g_best_acceleration = acceleration[x.first];
	            break;
	        }
	    }
	    
	}
	
	void update_EA(){
	    //vector<vector<int> > front1; //for plotting
	    
	    vector<vector<int> > EA_F_obj_values;
	    vector<vector<int> > EA_F_seq;
	    vector<vector<int> > EA_F_pa;
	    int temp_index = 0;
	    for(int i=0;i<EA_seq.size();i++){
	        EA_F_seq.push_back(EA_seq[i]);
	        EA_F_pa.push_back(EA_pa[i]);
	        temp_index++;
	        EA_F_obj_values.push_back(EA_obj_values[i]);
	    }
	    EA_obj_values.clear();
	    EA_seq.clear();
	    EA_pa.clear();
	    for(auto x:F_values){
	        EA_F_pa.push_back(process_alternatives[x.first]);
	        EA_F_seq.push_back(population[x.first]);
	        EA_F_obj_values.push_back(x.second);
	        temp_index++;
	    }
	    
	    unordered_set<int> s;
	    for(int i=0;i<temp_index;i++){
	    	for(int j=i+1;j<temp_index;j++){
	    		if(s.find(j)!=s.end()) continue;
	    		if(EA_F_seq[j]==EA_F_seq[i] && EA_F_pa[j]==EA_F_pa[i]){
	            	s.insert(j);
				}
			}
	    }
	    
	    for(int i=0;i<temp_index;i++){
	        int n = 0;
	        for(int j=0;j<temp_index;j++){
	            if(i==j || s.find(j)!=s.end()) continue;
	            
	            if(is_dominating(EA_F_obj_values[j], EA_F_obj_values[i])){
	                n = n + 1;
	            }
	        }
	        if(n==0 && s.find(i)==s.end()){
	            EA_seq.push_back(EA_F_seq[i]);
	            EA_pa.push_back(EA_F_pa[i]);
	            EA_obj_values.push_back(EA_F_obj_values[i]);
	        }
	    }
	    //plot_EA();
	}
	
	
	void print_all_solution(){
		int index = 0;
	    cout<<"\n\n\n============PRINTING ALL SOLUTIONS FROM THE EXTERNAL ARCHIVE=============="<<endl<<endl<<endl;
	    unordered_set<int> s;
	    for(int i=0;i<EA_seq.size();i++){
	    	/*for(int j=0;j<EA_seq.size();j++){
	    		if(i==j || s.find(j)!=s.end()) continue;
	    		if(EA_seq[j]==EA_seq[i] && EA_pa[j]==EA_pa[i]){
	            	s.insert(j);
				}
			}
	    	if(s.find(i)!=s.end()) continue;*/
	        cout<<"\n\nSOLUTION NUMBER : "<<index+1<<endl;
	        index++;
	        print_solution(EA_obj_values[i][0], EA_obj_values[i][1], EA_seq[i], EA_pa[i]);
	    }
	}
	
	void escape_local_minima(){
        //assuming if the value of a student is not changing even after 30% of iterations, its replaced with a random sequence
        bool changed;
        for(int i=0;i<p;i++){
            changed = false;
            if(counter[i]>=floor(0.2*T)){
                changed = true;
                counter[i] = 0;
                population[i] = generate_sequence_randomly(pre_graph);
                process_alternatives[i] = generate_process_alternative();
                obj_value[i][0] = objective_function(population[i], process_alternatives[i]);
                obj_value[i][1] = energy_calculation(population[i], process_alternatives[i], obj_value[i][0]);
            }
        }
        if(changed){
            update_EA();
        }
    }
};


int main()
{
   srand(time(0));	
   int t = 1;
   string s = "C:/Users/user/OneDrive/Desktop/CAPSTONE PROJECT/CODES/DATA/input_53_10.txt";
   AOA_ABC object;
    cout<<"Enter number of iterations : ";
    cin>>object.T;
    cout<<"Enter population size : ";
    cin>>object.p;
   object.get_input(s);
   object.generate_population();
   int T = object.T;
   object.print_iteration(1);
   for(int t=2;t<=T;t++){
       object.update_population(t);
       object.print_iteration(t);
   }
   object.update_EA();
   object.print_all_solution();
   return 0;
}
