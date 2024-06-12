#include <cam_processing.h>

CamProcess::CamProcess(std::string cam_topic, ) : it_(node_) {
  cam_sub_ = it_.subscribe(cam_topic, 1, &CamProcess::CamCallback, this);
}