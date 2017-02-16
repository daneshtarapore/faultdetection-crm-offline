#include <list>
#include <assert.h>

/******************************************************************************/
/******************************************************************************/

#include "featurevectorsinrobotagent.h"

/******************************************************************************/
/******************************************************************************/

void PrintFvDistribution(t_listFVsSensed &listFVsSensed)
{
    std::cout << "Number of entries in FV distribution is " << listFVsSensed.size() << std::endl;

    t_listFVsSensed::iterator itd = listFVsSensed.begin();
    while(itd != listFVsSensed.end())
    {
        std::cout << "FV " << itd->uFV << " number of robots " << itd->fRobots << " state " << itd->uMostWantedState << std::endl;
        ++itd;
    }

    std::cout << "Finished printing FvDistribution" << std::endl;
}

/******************************************************************************/
/******************************************************************************/

void PrintFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds)
{
    std::cout << "Number of entries in detailed map is " << listMapFVsToRobotIds.size() << std::endl;

    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end())
    {
        std::cout << "Observed robot id " << itd->uRobotId << " has fv " << itd->uFV << ", first time sensed is " << itd->fTimeSensed
                  << "range " << itd->uRange << " num s-m obs " << itd->uNumobs_sm << ", num s-nm obs " << itd->uNumobs_nsm
                  << ", num m obs " << itd->uNumobs_m
                  << std::endl;
        ++itd;
    }

    std::cout << "Finished printing FvToRobotIdMap" << std::endl;
}

/******************************************************************************/
/******************************************************************************/

void UpdaterFvDistribution(t_listFVsSensed &listFVsSensed, t_listMapFVsToRobotIds &listMapFVsToRobotIds, size_t num_observed_robots)
{

    t_listFVsSensed::iterator it_dist;

    listFVsSensed.clear();

    // updated listFVsSensed distribution with the most recent number of robots for different FVs.
    for(t_listMapFVsToRobotIds::iterator it_map = listMapFVsToRobotIds.begin(); it_map != listMapFVsToRobotIds.end(); ++it_map)
    {
        double increment = 1.0f;

        bool b_EntryInserted(false);

        // check if fv is in listFVsSensed
        // if so, update the value it holds by increment
        // if not insert it (while keeping list sorted based on fv) and initialize its value by increment
        for (it_dist = listFVsSensed.begin(); it_dist != listFVsSensed.end(); ++it_dist)
        {
            if(it_dist->uFV == it_map->uFV)
            {
                // if fv is already present
                it_dist->fRobots += increment;
                b_EntryInserted = true;
                break;
            }

            if(it_dist->uFV > it_map->uFV)
            {   // we assume the list is kept sorted.
                // if fv is absent
                listFVsSensed.insert(it_dist, StructFVsSensed(it_map->uFV, increment));
                b_EntryInserted = true;
                break;
            }
        }

        if(b_EntryInserted)
            continue;

        // when the list is empty or item is to be inserted in the end
        listFVsSensed.push_back(StructFVsSensed(it_map->uFV, increment));
    }

    size_t sum_robots = 0;
    for (it_dist = listFVsSensed.begin(); it_dist != listFVsSensed.end(); ++it_dist)
        sum_robots += (size_t)it_dist->fRobots;

    assert(sum_robots == num_observed_robots);
}

/******************************************************************************/
/******************************************************************************/

void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                          unsigned int fv, unsigned robotId)
{
    t_listMapFVsToRobotIds::iterator itd;

    // check if robotId is in listMapFVsToRobotIds
    // if so, than update its fv if timesensed is more recent.
    // if not, than add its fv to the list

    bool robot_present(false);
    for (itd = listMapFVsToRobotIds.begin(); itd != listMapFVsToRobotIds.end(); ++itd)
    {
        if((*itd).uRobotId == robotId)
        {
            robot_present = true;
            std::cout << "This should not occur. A robot id should appear no more than once in listMapFVsToRobotIds";
            assert(-1);

//            // if the robot id is already present, add if the new information is more recent
//            if(timesensed > (*itd).fTimeSensed)
//            {
//                (*itd).uRobotId = robotId;
//                (*itd).uFV = fv;
//            }
            return;
        }
    }

    if(!robot_present) // if the list is empty or the robot with given id is not present in the list
        listMapFVsToRobotIds.push_back(DetailedInformationFVsSensed(robotId, fv));
}

/******************************************************************************/
/******************************************************************************/
