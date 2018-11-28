

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "util/NumType.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

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
  }

  // virtual ~SampleOutputWrapper()
  // {
  //     printf("OUT: Destroyed SampleOutputWrapper\n");
  // }

  virtual void publishKeyframes(std::vector<FrameHessian*>& frames, bool final, CalibHessian* HCalib) override
  {
    std::vector<PointHessian*> accumulated_pointcloud;
    // std::vector<std::shared_ptr<dso::pointHessians>> accumulated_pointcloud;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    for (FrameHessian* f : frames) {
      //   printf("OUT: KF %d (%s) (id %d, tme %f): %d active, %d marginalized, %d immature points. CameraToWorld:\n",
      //          f->frameID,
      //          final ? "final" : "non-final",
      //          f->shell->incoming_id,
      //          f->shell->timestamp,
      //          (int)f->pointHessians.size(),
      //          (int)f->pointHessiansMarginalized.size(),
      //          (int)f->immaturePoints.size());
      //   std::cout << f->shell->camToWorld.matrix() << "\n";

      //   int maxWrite = 5;
      for (PointHessian* p : f->pointHessians) {
        //     printf("OUT: Example Point x=%.1f, y=%.1f, idepth=%f, idepth std.dev. %f, %d inlier-residuals\n",
        //            p->u,
        //            p->v,
        //            p->idepth_scaled,
        //            sqrt(1.0f / p->idepth_hessian),
        //            p->numGoodResiduals);
        //     maxWrite--;
        //     if (maxWrite == 0)
        //       break;
        Vec4 pt_cam;
        // TODO: verify that idepth == inverse depth
        pt_cam[0] = (p->u - HCalib->cxl()) / HCalib->fxl() / p->idepth;
        pt_cam[1] = (p->v - HCalib->cyl()) / HCalib->fyl() / p->idepth;
        pt_cam[2] = 1 / p->idepth;
        pt_cam[3] = 1;
        std::cout << "Pt before: " << pt_cam << std::endl;
        pt_cam = f->shell->camToWorld.matrix() * pt_cam;
        std::cout << "Pt after: " << pt_cam << std::endl;
        pcl::PointXYZRGB pt_w(255, 255, 255);
        //   f->PRE_camToWorld  // sophus SE3 tf
        //   Matx44f T_w_cam = f->sh
        pt_w.x = pt_cam[0];
        pt_w.y = pt_cam[1];
        pt_w.z = pt_cam[2];
        cloud.push_back(pt_w);
      }
      // TODO: watch out, f->pointHessians is a normal ptr, there may be some dereferencing issues. Might have to make
      // copies instead. for each point, backproject into KF and transform using that KF pose

      //   accumulated_pointcloud.insert(accumulated_pointcloud.end(), f->pointHessians.begin(),
      //   f->pointHessians.end());
    }

    // TODO: extract point cloud near current pose. Maybe use KNN library? PCL?
    // std::string current_pose =

    sensor_msgs::PointCloud2 ros_pointcloud;
    pcl::toROSMsg(cloud, ros_pointcloud);
    ros_pointcloud.header.frame_id = "world";

    // loop over all KFs (and accumulate points inside a certain radius?)
    std_msgs::String msg;
    msg.data = "Number of accumulated points: " + std::to_string(cloud.size()) + "\n";

    // TODO: transform to ROS point cloud msg

    pointcloud_info_pub.publish(msg);
    pointcloud_pub.publish(ros_pointcloud);
  }

private:
  ros::Publisher pointcloud_pub;
  ros::Publisher pointcloud_info_pub;
};

} // namespace IOWrap
} // namespace dso