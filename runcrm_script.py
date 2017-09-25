#!/usr/bin/python

import os
import glob

offline_crm='/home/danesh/argos3-epuck/faultdetection-crm-offline/build/faultdetection-crm-offline'
raw_fv_dir="/home/danesh/argos3-epuck/argos3-foraging/conf/SimulationsOfPhysicalSwarmExpts"
fv_pattern="SWARM*_FAULT*_*.fvlog_PropFV"
os.chdir(raw_fv_dir)
tmp_exp_lists = glob.glob(fv_pattern)

#for affinity_value in ['0.3' '0.4' '0.5' '0.6' '0.7']:
#    os.mkdir("affinity_" + affinity_value)    


exp_lists=[]
for exp in tmp_exp_lists:
    exp = exp[:-13]
    #print(exp)
    exp_lists.append(exp)
    
exp_lists = list(set(exp_lists)) # remove duplicates from list

swarm_behavs = []
fault_types = []
rand_seeds = []

#print exp_lists

counter = 0
for exp in exp_lists:
    k = exp.find('_FAULT_', 0, len(exp))
    swarm_behavs.append(exp[:k])
    swarm_behav = exp[:k]
    #print(swarm_behav)
    fault_types.append(exp[k+1:len(exp)-4])
    fault_type = exp[k+1:len(exp)-4]
    #print(fault_type)
    rand_seeds.append(exp[len(exp)-3:])
    rand_seed = exp[len(exp)-3:]
    #print(rand_seed)
    #print("Running " + str(counter) + " of " + str(len(exp_lists)))

    for affinity_counter,affinity_value in enumerate(['0.3', '0.35', '0.4', '0.45', '0.5', '0.55', '0.6', '0.65', '0.7', '0.75']):
        command_string = "sem --no-notice -j 6 " + offline_crm + " " + swarm_behav + " " + fault_type + " " + rand_seed + " " + affinity_value  
        #print(command_string)
        x1 = os.system(command_string)
    
    if x1 == 0:
        print("Run started " + command_string)
    else:
        print("Could not run " + command_string)

    counter = counter + 1    
