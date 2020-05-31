//
// Created by jieming on 28.04.20.
//

#include "Config.h"

void Config::setParameterFile(const std::string& filename) {
    if(config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config);

    config_->file_ = cv::FileStorage(filename, cv::FileStorage::READ );

    if(config_->file_.isOpened() == false){
        std::cerr << "parameter file" << filename << "does not exist." << std::endl;
        config_->file_.release();
        return;
    }
}

Config::~Config() {
    if(file_.isOpened())
        file_.release();
}

shared_ptr<Config> Config::config_ = nullptr;
