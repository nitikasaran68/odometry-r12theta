//
//  main.cpp
//  plot pose
//
//  Created by Himani Arora on 13/10/15.
//  Copyright (c) 2015 Himani Arora. All rights reserved.
//

#include <iostream>
#include <fstream>
using namespace std;

int main()
{
    ifstream read_file;
    ofstream write_file;
    
    read_file.open("/Users/himaniarora/Desktop/poses.txt");
    write_file.open("/Users/himaniarora/Desktop/poses_modified.txt");
    float tx,ty,tz,w1_cos,w2_cos,w3_cos,scale;
    int frame_num;
    float multiplier=0.05f;
    
    while(!read_file.eof())
    {
        read_file>>frame_num>>tx>>ty>>tz>>w1_cos>>w2_cos>>w3_cos>>scale;
        printf("%d , %f, %f, %f, %f, %f, %f, %f\n", frame_num, tx, ty, tz, w1_cos, w2_cos, w3_cos, scale);
        write_file<<frame_num<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<multiplier*w1_cos<<" "<<multiplier*w2_cos<<" "<<multiplier*w3_cos<<" "<<scale<<"\n";
        
    }
    read_file.close();
    write_file.close();
    
}
