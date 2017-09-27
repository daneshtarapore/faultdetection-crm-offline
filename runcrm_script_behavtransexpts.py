#!/usr/bin/python

import os

offline_crm='/home/danesh/argos3-epuck/faultdetection-crm-offline/build/faultdetection-crm-offline'
swarm_behav = "SWARM_AGGREGATION"
affinity_value = "0.55"


trans_behav = "DISPERSION_0.1"
for rand_seed in ['111', '222', '333', '444', '555']:
    command_string = "sem --no-notice -j 7 " + offline_crm + " " + swarm_behav + " " + trans_behav + " " + rand_seed
    print command_string

    x1 = os.system(command_string)

    if x1 == 0:
        print "Run started " + command_string
    else:
        print "Could not run " + command_string


trans_behav = "DISPERSION_0.2"
for rand_seed in ['111', '222', '333', '444', '555']:
    command_string = "sem --no-notice -j 7 " + offline_crm + " " + swarm_behav + " " + trans_behav + " " + rand_seed
    print command_string

    x1 = os.system(command_string)

    if x1 == 0:
        print "Run started " + command_string
    else:
        print "Could not run " + command_string


trans_behav = "DISPERSION_1.0"
for rand_seed in ['111', '222', '333', '444', '555']:
    command_string = "sem --no-notice -j 7 " + offline_crm + " " + swarm_behav + " " + trans_behav + " " + rand_seed
    print command_string

    x1 = os.system(command_string)

    if x1 == 0:
        print "Run started " + command_string
    else:
        print "Could not run " + command_string
