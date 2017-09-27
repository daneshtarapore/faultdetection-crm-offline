#!/usr/bin/python

# extract and combine all robot proprioceptive FVs into a single file for ease of visibility

import os
import glob

raw_fv_dir="/home/danesh/papers/2017/fd_realrobotswarm/robotfvdata/RawPropFVs"
fv_pattern="./[0-9][0-9][0-9]_SWARM*_FAULT*_*.fvlog_PropFV"
os.chdir(raw_fv_dir)
tmp_exp_lists = glob.glob(fv_pattern)


#for affinity_value in ['0.3' '0.4' '0.5' '0.6' '0.7']:
#    os.mkdir("affinity_" + affinity_value)    


exp_lists=[]
for exp in tmp_exp_lists:
    exp = exp[6:-13]
    #print(exp)
    exp_lists.append(exp)
    
exp_lists = list(set(exp_lists)) # remove duplicates from list

swarm_behavs = []
fault_types = []
rand_seeds = []

#print exp_lists

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


    filename_string = swarm_behav + "_" + fault_type + "_" + rand_seed + ".fvlog_PropFV"
    print("Reading " + "201_"+filename_string)

    f201 = open("201_"+filename_string,"r"); f201_lines = f201.readlines()
    f202 = open("202_"+filename_string,"r"); f202_lines = f202.readlines()
    f206 = open("206_"+filename_string,"r"); f206_lines = f206.readlines()
    f207 = open("207_"+filename_string,"r"); f207_lines = f207.readlines()
    f208 = open("208_"+filename_string,"r"); f208_lines = f208.readlines()
    f209 = open("209_"+filename_string,"r"); f209_lines = f209.readlines()
    f210 = open("210_"+filename_string,"r"); f210_lines = f210.readlines()        


    steps = [line.split(" ")[0] for line in f201_lines]
    fv0  = [line.split(" ")[1][:-1] for line in f201_lines]
    fv1  = [line.split(" ")[1][:-1] for line in f202_lines]
    fv2  = [line.split(" ")[1][:-1] for line in f206_lines]
    fv3  = [line.split(" ")[1][:-1] for line in f207_lines]
    fv4  = [line.split(" ")[1][:-1] for line in f208_lines]
    fv5  = [line.split(" ")[1][:-1] for line in f209_lines]
    fv6  = [line.split(" ")[1][:-1] for line in f210_lines]        

    f = open(filename_string,"w");

    for step in steps:
        step_i = int(step)
        f.write(step + "\t201 " + fv0[step_i] + "\t202 " + fv1[step_i] + "\t206 " + fv2[step_i] + "\t207 " + fv3[step_i] + "\t208 " + fv4[step_i] + "\t209 " + fv5[step_i] + "\t210 " + fv6[step_i] +"\n")
    


for trans_behav in ["DISPERSION_0.1", "DISPERSION_0.2", "DISPERSION_1.0"]:
    for rand_seed in ['111', '222', '333', '444', '555']:
        filename_string = swarm_behav + "_" + trans_behav + "_" + rand_seed + ".fvlog_PropFV"

        print("Reading " + "201_"+filename_string)

        f201 = open("201_"+filename_string,"r"); f201_lines = f201.readlines()
        f202 = open("202_"+filename_string,"r"); f202_lines = f202.readlines()
        f206 = open("206_"+filename_string,"r"); f206_lines = f206.readlines()
        f207 = open("207_"+filename_string,"r"); f207_lines = f207.readlines()
        f208 = open("208_"+filename_string,"r"); f208_lines = f208.readlines()
        f209 = open("209_"+filename_string,"r"); f209_lines = f209.readlines()
        f210 = open("210_"+filename_string,"r"); f210_lines = f210.readlines()        

        steps = [line.split(" ")[0] for line in f201_lines]
        fv0  = [line.split(" ")[1][:-1] for line in f201_lines]
        fv1  = [line.split(" ")[1][:-1] for line in f202_lines]
        fv2  = [line.split(" ")[1][:-1] for line in f206_lines]
        fv3  = [line.split(" ")[1][:-1] for line in f207_lines]
        fv4  = [line.split(" ")[1][:-1] for line in f208_lines]
        fv5  = [line.split(" ")[1][:-1] for line in f209_lines]
        fv6  = [line.split(" ")[1][:-1] for line in f210_lines]        

        f = open(filename_string,"w");

        for step in steps:
            step_i = int(step)
            f.write(step + "\t201 " + fv0[step_i] + "\t202 " + fv1[step_i] + "\t206 " + fv2[step_i] + "\t207 " + fv3[step_i] + "\t208 " + fv4[step_i] + "\t209 " + fv5[step_i] + "\t210 " + fv6[step_i] +"\n")
