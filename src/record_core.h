//
// Created by moi on 23-6-8.
//

#ifndef ROBOKIT_RECORD_CORE_H
#define ROBOKIT_RECORD_CORE_H
#include <fstream>
#include <iostream>
#include <stdio.h>
#include "vector"

using namespace std;
class record_core {
public:
	record_core();
	~record_core();
	void set_folder_location_path(const std::string path);
private:
	vector<ofstream> record_manger_;
	string folder_location_path_;
};


#endif //ROBOKIT_RECORD_CORE_H
