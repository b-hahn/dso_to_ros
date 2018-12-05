

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "util/NumType.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
#include "FullSystem/ImmaturePoint.h"


namespace dso {
namespace IOWrap {
class ROSOutputWrapper : public SampleOutputWrapper
{
public:
  inline ROSOutputWrapper(ros::NodeHandle& nh)
  {
    printf("OUT: Created ROSOutputWrapper\n");
    pointcloud_info_pub = nh.advertise<std_msgs::String>("pointcloud_info", 1000);
    pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("pointcloud", 1000);
    printf("OUT: generated publisher!\n");

    numPCL = 0;
    isSavePCL = true;
    isPCLfileClose = false;

    pclFile.open(strTmpFileName);
  }

  ~ROSOutputWrapper()
  {
    if (pclFile.is_open()) {
      pclFile.close();
    }

    printf("OUT: Destroyed SampleOutputWrapper\n");
  }


  virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) override
        {
            float fx, fy, cx, cy;
            float fxi, fyi, cxi, cyi;
            //float colorIntensity = 1.0f;
            fx = HCalib->fxl();
            fy = HCalib->fyl();
            cx = HCalib->cxl();
            cy = HCalib->cyl();
            fxi = 1 / fx;
            fyi = 1 / fy;
            cxi = -cx / fx;
            cyi = -cy / fy;

            Eigen::Matrix<double,3,3> T_world_dso;
            T_world_dso << 0, 0, 1,
                -1, 0, 0,
                 0, -1, 0;

            if (final) {
              for (FrameHessian* f : frames) {
                if (f->shell->poseValid) {
                  auto const& m = f->shell->camToWorld.matrix3x4();

                  // use only marginalized points.
                  auto const& points = f->pointHessiansMarginalized;

                  for (auto const* p : points) {
                    float depth = 1.0f / p->idepth;
                    auto const x = (p->u * fxi + cxi) * depth;
                    auto const y = (p->v * fyi + cyi) * depth;
                    auto const z = depth * (1 + 2 * fxi);

                    Vec4 camPoint(x, y, z, 1.f);
                    Vec3 worldPoint = T_world_dso *  m * camPoint * 20.6128;

                    if (isSavePCL && pclFile.is_open()) {
                      isWritePCL = true;

                      pclFile << worldPoint[0] << " " << worldPoint[1] << " " << worldPoint[2] << "\n";

                      if (numPCL % 1000 == 0) {
                        printf("[%d] Point Cloud Coordinate> X: %.2f, Y: %.2f, Z: %.2f\n",
                               numPCL,
                               worldPoint[0],
                               worldPoint[1],
                               worldPoint[2]);
                      }

                      numPCL++;
                      isWritePCL = false;
                    } else {
                      if (!isPCLfileClose) {
                        if (pclFile.is_open()) {
                          pclFile.flush();
                          pclFile.close();
                          isPCLfileClose = true;
                        }
                      }
                    }
                  }
                }
              }
            }


}

private:
  ros::Publisher pointcloud_pub;
  ros::Publisher pointcloud_info_pub;
  std::ofstream pclFile;

  // Added By Yo Han.
  int numPCL = 0;
  bool isSavePCL = true;
  bool isWritePCL = false;
  bool isPCLfileClose = false;
  std::string strTmpFileName = "pcl_data_tmp.pcd";
  std::string strSaveFileName = "pcl_data.pcd";

  double distance_threshold = 10000.;
};

} // namespace IOWrap
} // namespace dso