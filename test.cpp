
#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"
#include "visualize.h"
#include "lookup.h"
#include<stdlib.h>
using namespace std;
using namespace HybridAStar;

void tracePath(const Node3D* path);

int main(){

    //nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid());
    CollisionDetection configurationSpace;
    
    float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
    //sleep(5);
    cout<<"lookup table started"<<endl;
    //Lookup::dubinsLookup(dubinsLookup);
    cout<<"lookup table complete"<<endl;
    int w = 100;
    int h = 100;
    int i;
    int data1[10000] = {0};
    //cout<<100;
    //sleep(5);
    //printf("a");
    cout<<data1[0]<<endl;
    cout<<*data1<<endl;
    for (int i=0; i<w*h; i++) {
        map->data.push_back(0);
    }
    //map->data= data1;
    for(i=20;i<80;i++){
        map->data[i*w + 50] = 100;
        //cout<<100;
    }
    //cout<<sizeof(map->data)<<endl;
    
    for (int i=0; i<w*h; i++) {
        if ((i%100)==0){
            printf("\n");
        }
        if (map->data[i]==0){printf("-");}
        if (map->data[i]==100){printf("*");}        
    }
    
    
    printf("\n");
    map->info.width=w;
    map->info.height=h;
    map->info.resolution=0.25;
    map->header.frame_id="odom";
    cout<<"node_start"<<endl;
    Node3D nStart(12.5, 5, 0, 0, 0, nullptr);
    cout<<"node_end"<<endl;
    Node3D nGoal(12.5, 15, 0, 0, 0, nullptr);

    int depth = Constants::headings;
    int length = w * h * depth;
    cout<<"memory allocation start"<<endl;
    
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[w * h]();
    
    
    configurationSpace.updateGrid(map);
    cout<<"setup complete"<<endl;
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, w, h, configurationSpace, dubinsLookup);
    
    tracePath(nSolution);
    
return(0);
}

void tracePath(const Node3D* node){
    if (node != nullptr){
        printf("x = %f, Y = %f\n",node->getX(),node->getY());
        tracePath(node->getPred());
    }
}
