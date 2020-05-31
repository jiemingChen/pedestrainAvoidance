//
// Created by jieming on 28.04.20.
//

#ifndef COLLISION_AVOIDANCE_CONFIG_H
#define COLLISION_AVOIDANCE_CONFIG_H

#include "collision_avoidance/header.h"

class Config {
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {}; //private constructor makes a singleton
public:
    ~Config();  // close the file when deconstructing
    static void setParameterFile(const std::string& filename);

    template<typename T>
    static T get(const std::string& key){
        return T( Config::config_->file_[key] );
    }
};


#endif //COLLISION_AVOIDANCE_CONFIG_H
