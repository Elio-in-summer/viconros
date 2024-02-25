#include "libmotioncapture/vicon.h"

// VICON
#include "ViconDataStreamSDK_CPP/DataStreamClient.h"

using namespace ViconDataStreamSDK::CPP;

namespace libmotioncapture {

  class MotionCaptureViconImpl
  {
  public:
    Client client;
    std::string version;
  };

  MotionCaptureVicon::MotionCaptureVicon(
    const std::string& hostname,
    bool enableObjects,
    bool enablePointcloud)
  {
    pImpl = new MotionCaptureViconImpl;

    // Try connecting...
    while (!pImpl->client.IsConnected().Connected) {
      pImpl->client.Connect(hostname);
    }

    if (enableObjects) {
      pImpl->client.EnableSegmentData();
    }
    if (enablePointcloud) {
      pImpl->client.EnableUnlabeledMarkerData();
    }

    // This is the lowest latency option
    pImpl->client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

    // Set the global up axis
    pImpl->client.SetAxisMapping(Direction::Forward,
                          Direction::Left,
                          Direction::Up); // Z-up

    // Discover the version number
    Output_GetVersion version = pImpl->client.GetVersion();
    std::stringstream sstr;
    sstr << version.Major << "." << version.Minor << "." << version.Point;
    pImpl->version = sstr.str();
  }

  MotionCaptureVicon::~MotionCaptureVicon()
  {
    delete pImpl;
  }

  const std::string& MotionCaptureVicon::version() const
  {
    return pImpl->version;
  }

  void MotionCaptureVicon::waitForNextFrame()
  {
    while (pImpl->client.GetFrame().Result != Result::Success) {
    }
  }

  void MotionCaptureVicon::getObjects(
    std::vector<Object>& result) const
  {
    result.clear();
    size_t count = pImpl->client.GetSubjectCount().SubjectCount;
    result.resize(count);
    for (size_t i = 0; i < count; ++i) {
      const std::string name = pImpl->client.GetSubjectName(i).SubjectName;
      getObjectByName(name, result[i]);
    }
  }

  void MotionCaptureVicon::getObjectByName(
    const std::string& name,
    Object& result) const
  {
    auto const translation = pImpl->client.GetSegmentGlobalTranslation(name, name);
    auto const quaternion = pImpl->client.GetSegmentGlobalRotationQuaternion(name, name);
    if (   translation.Result == Result::Success
        && quaternion.Result == Result::Success
        && !translation.Occluded
        && !quaternion.Occluded) {

      Eigen::Vector3f position(
        translation.Translation[0] / 1000.0,
        translation.Translation[1] / 1000.0,
        translation.Translation[2] / 1000.0);

      Eigen::Quaternionf rotation(
        quaternion.Rotation[3], // w
        quaternion.Rotation[0], // x
        quaternion.Rotation[1], // y
        quaternion.Rotation[2]  // z
        );
      result = Object(name, position, rotation,(pImpl->client.GetFrameNumber().FrameNumber));
      //lss
        Eigen::MatrixXf velocityacceleration=getVelocityAcceleration(result,  pImpl->client.GetFrameRate().FrameRateHz);
        Eigen::Vector3f euler=getEuler(rotation);
        result.setVelocity(velocityacceleration);
        result.setEuler(euler);
       //lss
    } else {
      result = Object(name);
    }
  }

    //lss
    Eigen::MatrixXf MotionCaptureVicon::getVelocityAcceleration(Object& result, double FrameRateHz) const
    {
        Eigen::MatrixXf velocityacceleration(4,3);
        float lastvel=0,currentvel=0;
        Eigen::Quaternionf lastangle(0,0,0,0),currentangle(0,0,0,0);
        velocityacceleration << 0,0,0,
                                0,0,0,
                                0,0,0,
                                0,0,0;
        historyObject.push_back(result);
        if(historyObject.size()>3)
        {
            historyObject.pop_front();
            std::list<Object>::iterator it=historyObject.begin();
            it++;
            // linear velocity
            velocityacceleration(0,0)=((historyObject.front().position())(0)-(historyObject.back().position())(0))*FrameRateHz/(historyObject.front().getFramenumber()-historyObject.back().getFramenumber());
            velocityacceleration(0,1)=((historyObject.front().position())(1)-(historyObject.back().position())(1))*FrameRateHz/(historyObject.front().getFramenumber()-historyObject.back().getFramenumber());
            velocityacceleration(0,2)=((historyObject.front().position())(2)-(historyObject.back().position())(2))*FrameRateHz/(historyObject.front().getFramenumber()-historyObject.back().getFramenumber());

            // linear acceleration
            lastvel=((historyObject.front().position())(0)-((*it).position())(0))*FrameRateHz/(historyObject.front().getFramenumber()-(*it).getFramenumber());
            currentvel=(((*it).position())(0)-(historyObject.back().position())(0))*FrameRateHz/((*it).getFramenumber()-historyObject.back().getFramenumber());
            velocityacceleration(1,0)= (lastvel-currentvel)*FrameRateHz/((*it).getFramenumber()-historyObject.back().getFramenumber());

            lastvel=((historyObject.front().position())(1)-((*it).position())(1))*FrameRateHz/(historyObject.front().getFramenumber()-(*it).getFramenumber());
            currentvel=(((*it).position())(1)-(historyObject.back().position())(1))*FrameRateHz/((*it).getFramenumber()-historyObject.back().getFramenumber());
            velocityacceleration(1,1)= (lastvel-currentvel)*FrameRateHz/((*it).getFramenumber()-historyObject.back().getFramenumber());

            lastvel=((historyObject.front().position())(2)-((*it).position())(2))*FrameRateHz/(historyObject.front().getFramenumber()-(*it).getFramenumber());
            currentvel=(((*it).position())(2)-(historyObject.back().position())(2))*FrameRateHz/((*it).getFramenumber()-historyObject.back().getFramenumber());
            velocityacceleration(1,2)= (lastvel-currentvel)*FrameRateHz/((*it).getFramenumber()-historyObject.back().getFramenumber());

            // angle speed
            Eigen::Quaternionf q1((float)(2*FrameRateHz/(historyObject.back().getFramenumber()-historyObject.front().getFramenumber())), 0, 0, 0);
            Eigen::Quaternionf temptanglespeed=historyObject.back().rotation() * historyObject.front().rotation().inverse()*q1;
            velocityacceleration(2,0)=temptanglespeed.x();
            velocityacceleration(2,1)=temptanglespeed.y();
            velocityacceleration(2,2)=temptanglespeed.z();

            // angle acceleration
            Eigen::Quaternionf q2((float)(2*FrameRateHz/(historyObject.back().getFramenumber()-(*it).getFramenumber())), 0, 0, 0);
            currentangle=historyObject.back().rotation() * (*it).rotation().inverse()*q2;
            Eigen::Quaternionf q3((float)(2*FrameRateHz/((*it).getFramenumber()-historyObject.front().getFramenumber())), 0, 0, 0);
            lastangle=(*it).rotation() * historyObject.front().rotation().inverse()*q3;

            velocityacceleration(3,0)=(currentangle.x()-lastangle.x())*FrameRateHz/(historyObject.back().getFramenumber()-(*it).getFramenumber());
            velocityacceleration(3,1)=(currentangle.y()-lastangle.y())*FrameRateHz/(historyObject.back().getFramenumber()-(*it).getFramenumber());
            velocityacceleration(3,2)=(currentangle.z()-lastangle.z())*FrameRateHz/(historyObject.back().getFramenumber()-(*it).getFramenumber());

        }
        return velocityacceleration;
    }

    Eigen::Vector3f MotionCaptureVicon::getEuler(Eigen::Quaternionf& quarternion) const
    {
        Eigen::Vector3f euler(0,0,0);
        euler(0)= atan2(2.0 * (quarternion.w() * quarternion.x() + quarternion.y() * quarternion.z()), 1.0 - 2.0 * (quarternion.x() * quarternion.x() + quarternion.y() * quarternion.y()));
        euler(1)= -asin(2.0 * (quarternion.z() * quarternion.x() - quarternion.w() * quarternion.y()));
        euler(2)= atan2(2.0 * (quarternion.w() * quarternion.z() + quarternion.x() * quarternion.y()), 1.0 - 2.0 * (quarternion.y() * quarternion.y() + quarternion.z() * quarternion.z()));
        return euler;
    }
    //lss
  void MotionCaptureVicon::getPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr result) const
  {
    result->clear();
    size_t count = pImpl->client.GetUnlabeledMarkerCount().MarkerCount;
    for(size_t i = 0; i < count; ++i) {
      Output_GetUnlabeledMarkerGlobalTranslation translation =
        pImpl->client.GetUnlabeledMarkerGlobalTranslation(i);
      result->push_back(pcl::PointXYZ(
        translation.Translation[0] / 1000.0,
        translation.Translation[1] / 1000.0,
        translation.Translation[2] / 1000.0));
    }
  }

  void MotionCaptureVicon::getLatency(
    std::vector<LatencyInfo>& result) const
  {
    result.clear();
    size_t latencyCount = pImpl->client.GetLatencySampleCount().Count;
    for(size_t i = 0; i < latencyCount; ++i) {
      std::string sampleName  = pImpl->client.GetLatencySampleName(i).Name;
      double      sampleValue = pImpl->client.GetLatencySampleValue(sampleName).Value;
      result.emplace_back(LatencyInfo(sampleName, sampleValue));
    }
  }

  bool MotionCaptureVicon::supportsObjectTracking() const
  {
    return true;
  }

  bool MotionCaptureVicon::supportsLatencyEstimate() const
  {
    return true;
  }

  bool MotionCaptureVicon::supportsPointCloud() const
  {
    return true;
  }
}
