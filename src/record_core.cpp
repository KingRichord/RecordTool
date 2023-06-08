
#include "record_core.h"
record_core::record_core() {

}
record_core::~record_core() {
	if(!record_manger_.empty())
	for (auto &record:record_manger_) {
		record.close();
	}
	record_manger_.clear();
}
void record_core::set_folder_location_path(const std::string path)
{
	folder_location_path_ =path;
	string folderPath = "path";
	string command;
	command = "mkdir -p " + folderPath;
	system(command.c_str());
}

