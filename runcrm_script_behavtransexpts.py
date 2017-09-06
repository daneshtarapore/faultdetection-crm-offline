#!/usr/bin/python

import os

offline_crm='~/lpuck/faultdetection-crm-offline/build/faultdetection-crm-offline'
swarm_behav = "SWARM_AGGREGATION"

trans_behav = "DISPERSION_0.1"
for rand_seed in ['111', '222', '333', '444', '555']:
    command_string = offline_crm + " " + swarm_behav + " " + trans_behav + " " + rand_seed
    print command_string

    x1 = os.system(command_string)

    if x1 == 0:
        print "Run started " + command_string
    else:
        print "Could not run " + command_string


trans_behav = "DISPERSION_0.2"
for rand_seed in ['111', '222', '333', '444', '555']:
    command_string = offline_crm + " " + swarm_behav + " " + trans_behav + " " + rand_seed
    print command_string

    x1 = os.system(command_string)

    if x1 == 0:
        print "Run started " + command_string
    else:
        print "Could not run " + command_string


trans_behav = "DISPERSION_1.0"
for rand_seed in ['111', '222', '333', '444', '555']:
    command_string = offline_crm + " " + swarm_behav + " " + trans_behav + " " + rand_seed
    print command_string

    x1 = os.system(command_string)

    if x1 == 0:
        print "Run started " + command_string
    else:
        print "Could not run " + command_string