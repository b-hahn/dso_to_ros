

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pointmatcher_ros/transform.h>


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
    hue_pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("hue_pointcloud", 1000);
    cam_pose_pub = nh.advertise<tf::tfMessage>("tf", 1000);
    printf("OUT: generated publisher!\n");
  }

  // virtual ~SampleOutputWrapper()
  // {
  //     printf("OUT: Destroyed SampleOutputWrapper\n");
  // }


  static float rgb_to_hue(uint32_t r, uint32_t g, uint32_t b)
  {
    float hue = 0;
    std::vector<float> rgb;
    rgb.push_back(r / 255.0f);
    rgb.push_back(g / 255.0f);
    rgb.push_back(b / 255.0f);

    uint8_t max_index = std::distance(rgb.begin(), std::max_element(rgb.begin(), rgb.end()));
    uint8_t min_index = std::distance(rgb.begin(), std::min_element(rgb.begin(), rgb.end()));
    
    float diff = (rgb[max_index] - rgb[min_index]);
    
    // avoid overflow error
    if (diff < 0.000001) {
        return 0;
    }

    // if red has the max value     
    if (0 == max_index) {
        hue = (rgb[1] - rgb[2]) / diff * 60;
    }       
    else if (1 == max_index) {
        hue = (2.0 + (rgb[2] - rgb[0]) / diff) * 60;
    }
    else if (2 == max_index) {
        hue = (4.0 + (rgb[0] - rgb[1]) / diff) * 60;
    }

    return hue >= 0 ? hue : hue + 360;

  }

  virtual void publishKeyframes(std::vector<FrameHessian*>& frames, bool final, CalibHessian* HCalib) override
  {
    if (true == final) {
        return;
    }

    // TODO: maybe reserve space for cloud here?
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::PointCloud<pcl::PointXYZI> hue_cloud;
    int num_pt_marg = 0;
    Mat44 const T_newestF_world =  frames.back()->shell->camToWorld.matrix().inverse();

    for (FrameHessian* f : frames) {
      // if (f != frames.begin()[1] && f != frames.begin()[2]) {
      //   continue;
      // }
      Mat44 T_newestF_currF = T_newestF_world * f->shell->camToWorld.matrix();
      // ROS_WARN("distance from frame %d to current frame %d: (%f, %f, %f) --> norm: %f and %f and %f\n",
      //          f->frameID,
      //          frames.back()->frameID,
      //          T_newestF_currF(0, 3),
      //          T_newestF_currF(1, 3),
      //          T_newestF_currF(2, 3),
      //          T_newestF_currF.block<3, 1>(0, 3).norm(),
      //          T_newestF_world.block<3, 1>(0, 3).norm(),
      //          f->shell->camToWorld.matrix().block<3, 1>(0, 3).norm());

      float cx = HCalib->cxl() /* * SCALE_C */;
      float cy = HCalib->cyl() /* * SCALE_C */;
      float fx = HCalib->fxl() /* * SCALE_F */;
      float fy = HCalib->fyl() /* * SCALE_C */;
      float fxi = 1/fx;
      float fyi = 1/fy;
      float cxi = -cx / fx;
      float cyi = -cy / fy;

      // // draw all cam poses
      // pcl::PointXYZRGB pt_w(255, 255, 255);
      // Vec4 cam_pt = {0, 0, 0, 1};
      // cam_pt = T_newestF_currF * cam_pt * rescale_factor;
      // pt_w.x = cam_pt[0];
      // pt_w.y = cam_pt[1];
      // pt_w.z = cam_pt[2];
      // cloud.push_back(pt_w);
      // std::cout << "pt_w: " << pt_w.x << " " << pt_w.y << " " << pt_w.z << std::endl;

      // for (PointHessian* p : f->pointHessians) {
      //   Vec4 pt_cam;
      //   // TODO: verify that idepth == inverse depth
      //   float depth = 1.0f / p->idepth;
      //   auto const x = (p->u * fxi + cxi) * depth;
      //   auto const y = (p->v * fyi + cyi) * depth;
      //   auto const z = depth * (1 + 2 * fxi);

      //   pt_cam[0] = x;
      //   pt_cam[1] = y;
      //   pt_cam[2] = z;
      //   pt_cam[3] = 1.f;
      //   Vec4 pt_world = T_newestF_currF * pt_cam * rescale_factor;

      //   pcl::PointXYZRGB pt_w(p->color_rgb[0], p->color_rgb[1], p->color_rgb[2]);
      //   // pcl::PointXYZRGB pt_w(255, 255, 255);
      //   pt_w.x = pt_world[0];
      //   pt_w.y = pt_world[1];
      //   pt_w.z = pt_world[2];
      //   cloud.push_back(pt_w);
      //   // std::cout << "pt_w: " << pt_w.x << " " << pt_w.y << " " << pt_w.z << " coming from " << p->u << " " << p->v
      //   // << " and p->idepth = " << p->idepth << " and color: ";
      //   // for (auto c : p->color) {
      //   //   std::cout << c << ",";
      //   // } 

      //   // std::cout << std::endl;
      // }

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

        pcl::PointXYZRGB pt_w(p->color_rgb[0], p->color_rgb[1], p->color_rgb[2]);

        // pcl::PointXYZRGB pt_w(13, 244, 77);
        pt_w.x = pt_world[0];
        pt_w.y = pt_world[1];
        pt_w.z = pt_world[2];
        cloud.push_back(pt_w);
        num_pt_marg++;

        float hue = rgb_to_hue(p->color_rgb[0], p->color_rgb[1], p->color_rgb[2]);
        pcl::PointXYZI pt_hue;
        pt_hue.x = pt_world[0];
        pt_hue.y = pt_world[1];
        pt_hue.z = pt_world[2];
        pt_hue.intensity = hue;
        hue_cloud.push_back(pt_hue);

        // std::cout << "pt_w: " << pt_w.x << " " << pt_w.y << " " << pt_w.z << " coming from " << p->u << " " << p->v
                  // << " and p->idepth = " << p->idepth << std::endl;
      }

      // for (PointHessian* p : f->pointHessiansOut) {
      //   Vec4 pt_cam;
      //   // TODO: verify that idepth == inverse depth
      //   float depth = 1.0f / p->idepth;
      //   auto const x = (p->u * fxi + cxi) * depth;
      //   auto const y = (p->v * fyi + cyi) * depth;
      //   auto const z = depth * (1 + 2*fxi);
        
      //   pt_cam[0] = x;
      //   pt_cam[1] = y;
      //   pt_cam[2] = z;
      //   pt_cam[3] = 1.f;
      //   Vec4 pt_world = T_newestF_currF * pt_cam * rescale_factor;

      //   // pcl::PointXYZRGB pt_w(p->color_rgb[0], p->color_rgb[1], p->color_rgb[2]);
      //   pcl::PointXYZRGB pt_w(255, 255, 255);
      //   pt_w.x = pt_world[0];
      //   pt_w.y = pt_world[1];
      //   pt_w.z = pt_world[2];
      //   cloud.push_back(pt_w);
      // }

      // for (ImmaturePoint* p : f->immaturePoints) {
      //   Vec4 pt_cam;
      //   float idepth = (p->idepth_max + p->idepth_min) * 0.5f;
      //   float depth = 1.0f / idepth;
      //   auto const x = (p->u * fxi + cxi) * depth;
      //   auto const y = (p->v * fyi + cyi) * depth;
      //   auto const z = depth * (1 + 2*fxi);
        
      //   pt_cam[0] = x;
      //   pt_cam[1] = y;
      //   pt_cam[2] = z;
      //   pt_cam[3] = 1.f;
      //   Vec4 pt_world = T_newestF_currF * pt_cam * rescale_factor;
      //   pcl::PointXYZRGB pt_w(p->color_rgb[0], p->color_rgb[1], p->color_rgb[2]);
      //   // pcl::PointXYZRGB pt_w(255, 255, 255);
      //   pt_w.x = pt_world[0];
      //   pt_w.y = pt_world[1];
      //   pt_w.z = pt_world[2];
      //   cloud.push_back(pt_w);

      //   // std::cout << "pt_w: " << pt_w.x << " " << pt_w.y << " " << pt_w.z << " coming from " << p->u << " " << p->v
      //   // << " and p->idepth = " << idepth << " and color: ";
      //   // for (auto c : p->color_rgb) {
      //   //   std::cout << std::to_string(c) << ", ";
      //   // }

      //   // std::cout << std::endl;
      // }

      // TODO: watch out, f->pointHessians is a normal ptr, there may be some dereferencing issues. Might have to make
      // copies instead. for each point, backproject into KF and transform using that KF pose

    } 
        
    ROS_WARN("num marginalized points: %d\n", num_pt_marg);

    // don't publish an empty point cloud message since this can cause issues downstream
    if (num_pt_marg == 0) {
      return;
    }

    sensor_msgs::PointCloud2 ros_pointcloud;
    pcl::toROSMsg(cloud, ros_pointcloud);
    ros_pointcloud.header.frame_id = "cam02";
    // ros_pointcloud.header.frame_id = "world";
    // ROS_WARN("pc timestamp: %f\n", ros_pointcloud.header.stamp.toSec());

    // get most current frame's timestamp
    double timestamp = frames.back()->shell->timestamp;
    double dpth = frames.back()->pointHessiansMarginalized[0]->idepth;
    ros::Time time(timestamp);
    // ros_pointcloud.header.stamp = ros::Time::now();  // TODO: use time of correct TF
    ros_pointcloud.header.stamp = time;  // TODO: use time of correct TF

    // loop over all KFs (and accumulate points inside a certain radius?)
    std_msgs::String msg;
    msg.data = "Number of accumulated points: " + std::to_string(cloud.size()) + "\n";

    ROS_WARN("Number of accumulated points: %lu\n", cloud.size());
    
    sensor_msgs::PointCloud2 hue_ros_pointcloud;
    pcl::toROSMsg(hue_cloud, hue_ros_pointcloud);
    hue_ros_pointcloud.header.frame_id = "cam02";
    hue_ros_pointcloud.header.stamp = time;  // TODO: use time of correct TF

    pointcloud_info_pub.publish(msg);
    pointcloud_pub.publish(ros_pointcloud);
    hue_pointcloud_pub.publish(hue_ros_pointcloud);
  }


  virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
  {
    // printf("OUT: Current Frame %d (time %f, internal ID %d). CameraToWorld:\n",
    //        frame->incoming_id,
    //        frame->timestamp,
    //        frame->id);
    // std::cout << frame->camToWorld.matrix3x4() << "\n";

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
    // tf::Transform transform(T_world_cam.block<3, 3>(0, 0), T_world_cam.block<3, 1>(0, 3));
    // transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
    // tf::Quaternion q;
    // q.setRPY(0, 0, msg->theta);
    // transform.setRotation(q);
    br.sendTransform(T_world_cam_msg);  // TODO: change timestamp to ROS time format
    // br.sendTransform(tf::StampedTransform(transform, frame->timestamp, "world", "cam02"));  // TODO: change timestamp to ROS time format
  }

private:
  ros::Publisher pointcloud_pub;
  ros::Publisher hue_pointcloud_pub;
  ros::Publisher pointcloud_info_pub;
  ros::Publisher cam_pose_pub;

  float rescale_factor = 1.0;
  
};

} // namespace IOWrap
} // namespace dso