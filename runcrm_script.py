#!/usr/bin/python

import os
import glob

offline_crm='/home/danesh/argos3-epuck/faultdetection-crm-offline/build/faultdetection-crm-offline'
raw_fv_dir="/home/danesh/papers/2017/fd_realrobotswarm/robotfvdata/RawPropFVs"
fv_pattern="./[0-9][0-9][0-9]_SWARM*_FAULT*_*.fvlog_PropFV"
os.chdir(raw_fv_dir)
tmp_exp_lists = glob.glob(fv_pattern)

exp_lists=[]
for exp in tmp_exp_lists:
    exp = exp[6:-13]
    #print exp
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
    #print swarm_behav 
    fault_types.append(exp[k+1:len(exp)-4])
    fault_type = exp[k+1:len(exp)-4]
    rand_seeds.append(exp[len(exp)-3:])
    rand_seed = exp[len(exp)-3:]

    print "Running " + str(counter) + " of " + str(len(exp_lists))
    command_string = "sem --no-notice -j 7 " + offline_crm + " " + swarm_behav + " " + fault_type + " " + rand_seed
    print command_string

    x1 = os.system(command_string)

    if x1 == 0:
        print "Run started " + command_string
    else:
        print "Could not run " + command_string

    counter = counter + 1    
