//
// Created by 57725 on 2022/10/25.
//
#include "src.h"

record::record(){
}


record::record(const char *path) {
    stream.open(path, std::ios::out);
    if (!stream.is_open())
    {
        std::cout << path << " open fail" << std::endl;
        exit(0);
    }
}

void record::createfile(int type) {
    char path[100];
    memset(path, 0, sizeof(path));

    if(type)
    {
        sprintf(path, "degl/s%d-d-i%d-%d.txt", scenario_idx, instance_idx, run_num);
    } else{
        sprintf(path, "degl/s%d-r-i%d-%d.txt", scenario_idx, instance_idx, run_num);
    }

    stream.open(path, std::ios::out);
    if (!stream.is_open())
    {
        std::cout << path << " open fail" << std::endl;
        exit(0);
    }
}

record::record(int type) {
    this->createfile(type);
}




record::~record() {
    stream.close();
}

void record::parse_data(double t, int cost, int line_num) {
    if (t - this->tpoint > 0.25 || t > 60 )
    {
        this->stream << line_num << " " << t << " " << cost << std::endl;
        this->tpoint = t;
    }
}