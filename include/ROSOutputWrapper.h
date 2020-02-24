

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pointmatcher_ros/transform.h>
#include <segmatch/point_color_semantics.hpp>


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
    cam_pose_pub = nh.advertise<tf::tfMessage>("tf", 1000);
    printf("OUT: generated publisher!\n");
  }


  virtual void publishKeyframes(std::vector<FrameHessian*>& frames, bool final, CalibHessian* HCalib) override
  {
    if (true == final) {
        return;
    }

    // TODO: maybe reserve space for cloud here?
    // pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointCloud<segmatch::PointColorSemantics> cloud;
    int num_pt_marg = 0;
    Mat44 const T_newestF_world =  frames.back()->shell->camToWorld.matrix().inverse();

    for (FrameHessian* f : frames) {
      Mat44 T_newestF_currF = T_newestF_world * f->shell->camToWorld.matrix();

      float cx = HCalib->cxl() /* * SCALE_C */;
      float cy = HCalib->cyl() /* * SCALE_C */;
      float fx = HCalib->fxl() /* * SCALE_F */;
      float fy = HCalib->fyl() /* * SCALE_C */;
      float fxi = 1/fx;
      float fyi = 1/fy;
      float cxi = -cx / fx;
      float cyi = -cy / fy;

      for (PointHessian* p : f->pointHessiansMarginalized) {
        Vec4 pt_cam;
        // TODO: verify that idepth == inverse depth
        float depth = 1.0f / p->idepth;
        auto const x = (p->u * fxi + cxi) * depth;
        auto const y = (p->v * fyi + cyi) * depth;
        auto const z = depth * (1 + 2*fxi);

        pt_cam[0] = x;
        pt_cam[1] = y;
        pt_cam[2] = z;
        pt_cam[3] = 1.f;
        Vec4 pt_world = T_newestF_currF * pt_cam * rescale_factor;

        // EDIT: use real color
        uint32_t rgb_uint = (((uint32_t)p->color_rgb[0] << 16 | (uint32_t)p->color_rgb[1] << 8 | (uint32_t)p->color_rgb[2]));
        
        // EDIT: debug hack below to display semantic segmentation color labels on points
        // uint32_t rgb_uint = (((uint32_t)p->semantics_rgb[0] << 16 | (uint32_t)p->semantics_rgb[1] << 8 | (uint32_t)p->semantics_rgb[2]));
        // EDIT: use fixed color for debugging
        // uint32_t rgb_uint = (((uint32_t)44 << 16 | (uint32_t)45 << 8 | (uint32_t)46));
        float rgb = *reinterpret_cast<float*>(&rgb_uint);
        segmatch::PointColorSemantics pt_w;
        pt_w.rgb = rgb;

        pt_w.semantics_rgb =
          (((uint32_t)p->semantics_rgb[0] << 16 | (uint32_t)p->semantics_rgb[1] << 8 | (uint32_t)p->semantics_rgb[2]));
        // TODO: currently points are assigned semantics_rgb value of (0, 127, 0) == 32512 in stereo_dso.
        // This causes problems in SegMap since that color doesn't correspond to any Mapillary Vistas class.
        // In reality, all point should be assigned a value so I'm not sure where those points come from. Investigate and remove hack below.
        if (pt_w.semantics_rgb == 32512) {
            LOG(INFO) << "skipping!";
            continue;
        }
        pt_w.x = pt_world[0];
        pt_w.y = pt_world[1];
        pt_w.z = pt_world[2];
        cloud.push_back(pt_w);
        num_pt_marg++;
      }

    ROS_WARN("num marginalized points: %d\n", num_pt_marg);

    // don't publish an empty point cloud message since this can cause issues downstream
    if (num_pt_marg == 0) {
      return;
    }

    sensor_msgs::PointCloud2 ros_pointcloud;
    pcl::toROSMsg(cloud, ros_pointcloud);
    ros_pointcloud.header.frame_id = "cam02";

    // get most current frame's timestamp
    double timestamp = frames.back()->shell->timestamp;
    double dpth = frames.back()->pointHessiansMarginalized[0]->idepth;
    ros::Time time(timestamp);
    ros_pointcloud.header.stamp = time;  // TODO: use time of correct TF

    // loop over all KFs (and accumulate points inside a certain radius?)
    std_msgs::String msg;
    msg.data = "Number of accumulated points: " + std::to_string(cloud.size()) + "\n";

    ROS_WARN("Number of accumulated points: %lu\n", cloud.size());
    pointcloud_info_pub.publish(msg);
    pointcloud_pub.publish(ros_pointcloud);
  }


  virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
  {

    // permutation matrix to get world coord system (DSO in cam coord system)
    Eigen::Matrix<double,4, 4> T_world_dso;
    T_world_dso << 0, 0, 1, 0,
                  -1, 0, 0, 0,
                   0, -1, 0, 0,
                   0, 0, 0, 1;

    Eigen::Matrix<double, 4, 4> T_world_cam = T_world_dso * frame->camToWorld.matrix()/* .inverse() */;
    ros::Time time(frame->timestamp);

    tf::StampedTransform T_world_cam_msg =
      PointMatcher_ros::eigenMatrixToStampedTransform<double>(T_world_cam, "world", "cam02", time);

    tf::TransformBroadcaster br;
    br.sendTransform(T_world_cam_msg);
  }

private:
  ros::Publisher pointcloud_pub;
  ros::Publisher pointcloud_info_pub;
  ros::Publisher cam_pose_pub;

  float rescale_factor = 1.0;

};

} // namespace IOWrap
} // namespace dso