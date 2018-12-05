

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

  // virtual void publishKeyframes(std::vector<FrameHessian*>& frames, bool final, CalibHessian* HCalib) override
  // {
  //   if (true == final) {
  //       return;
  //   }

  //   // TODO: maybe reserve space for cloud here?
  //   pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //   int num_pt_marg = 0;
  //   Mat44 const T_newestF_world =  frames.back()->shell->camToWorld.matrix().inverse();
  //   // Mat44 T_world_newestF = frames.back()->shell->camToWorld.matrix();
  //   // std::cout << "T_world_newestF * T_newestF_world:\n" << T_world_newestF * T_newestF_world << std::endl;

  //   for (FrameHessian* f : frames) {
  //     Mat44 T_newestF_currF = T_newestF_world * f->shell->camToWorld.matrix();
  //     // std::cout << "T_newestF_world:\n" << T_newestF_world << std::endl;
  //     // std::cout << "f->shell->camToWorld.matrix():\n" << f->shell->camToWorld.matrix() << std::endl;
  //     // std::cout << "T_newestF_currF:\n" << T_newestF_currF << std::endl;
  //       // printf("OUT: KF %d (%s) (id %d, tme %f): %d active, %d marginalized, %d immature points. CameraToWorld:\n",
  //       //        f->frameID,
  //       //        final ? "final" : "non-final",
  //       //        f->shell->incoming_id,
  //       //        f->shell->timestamp,
  //       //        (int)f->pointHessians.size(),
  //       //        (int)f->pointHessiansMarginalized.size(),
  //       //        (int)f->immaturePoints.size());

  //     float cx = HCalib->cxl() /* * SCALE_C */;
  //     float cy = HCalib->cyl() /* * SCALE_C */;
  //     float fx = HCalib->fxl() /* * SCALE_F */;
  //     float fy = HCalib->fyl() /* * SCALE_C */;
  //     float fxi = 1/fx;
  //     float fyi = 1/fy;
  //     float cxi = -cx / fx;
  //     float cyi = -cy / fy;

  //     // std::cout << "cam calib: " << fx << " " << fy << " " << cx << " " << cy << std::endl;
  //     // std::cout << "cam calib inv: " << fxi << " " << fyi << " " << cxi << " " << cyi << std::endl;

  //     for (PointHessian* p : f->pointHessians) {
  //       Vec4 pt_cam;
  //       // TODO: verify that idepth == inverse depth
  //       float depth = 1.0f / p->idepth;
  //       auto const x = (p->u * fxi + cxi) * depth;
  //       auto const y = (p->v * fyi + cyi) * depth;
  //       auto const z = depth * (1 + 2 * fxi);

  //       pt_cam[0] = x;
  //       pt_cam[1] = y;
  //       pt_cam[2] = z;
  //       pt_cam[3] = 1.f;
  //       // std::cout << "pt_cam: " << pt_cam[0] << " " << pt_cam[1] << " " << pt_cam[2]
  //       // << " " << pt_cam[3] << " coming from " << p->u << " " << p->v << " and p->idepth = " << p->idepth <<
  //       // std::endl;
  //       // pt_cam[0] = (p->u - cx) / fx / p->idepth;
  //       // pt_cam[1] = (p->v - cy) / fy / p->idepth;
  //       // pt_cam[2] = 1 / p->idepth;
  //       // pt_cam[3] = 1;
  //       // pt_cam = f->shell->camToWorld.matrix() * pt_cam;
  //       // if (num_pt_marg % 100 == 0) {
  //       //   std::cout << "f->shell->camToWorld.matrix3x4(): " << f->shell->camToWorld.matrix3x4() << std::endl;
  //       // }
  //       Vec4 pt_world = T_newestF_currF * pt_cam * 20.6128;

  //       pcl::PointXYZRGB pt_w(255, 255, 255);
  //       pt_w.x = pt_world[0];
  //       pt_w.y = pt_world[1];
  //       pt_w.z = pt_world[2];
  //       cloud.push_back(pt_w);
  //       // std::cout << "pt_w: " << pt_w.x << " " << pt_w.y << " " << pt_w.z << " coming from " << p->u << " " << p->v
  //       // << " and p->idepth = " << p->idepth << std::endl;
  //     }

  //     for (PointHessian* p : f->pointHessiansMarginalized) {
  //       Vec4 pt_cam;
  //       // TODO: verify that idepth == inverse depth
  //       float depth = 1.0f / p->idepth;
  //       auto const x = (p->u * fxi + cxi) * depth;
  //       auto const y = (p->v * fyi + cyi) * depth;
  //       auto const z = depth * (1 + 2*fxi);
        
  //       pt_cam[0] = x;
  //       pt_cam[1] = y;
  //       pt_cam[2] = z;
  //       pt_cam[3] = 1.f;
  //       // std::cout << "pt_cam: " << pt_cam[0] << " " << pt_cam[1] << " " << pt_cam[2]
  //                 // << " " << pt_cam[3] << " coming from " << p->u << " " << p->v << " and p->idepth = " << p->idepth << std::endl;
  //       // pt_cam[0] = (p->u - cx) / fx / p->idepth;
  //       // pt_cam[1] = (p->v - cy) / fy / p->idepth;
  //       // pt_cam[2] = 1 / p->idepth;
  //       // pt_cam[3] = 1;
  //       // pt_cam = f->shell->camToWorld.matrix() * pt_cam;
  //       // if (num_pt_marg % 100 == 0) {
  //       //   std::cout << "f->shell->camToWorld.matrix3x4(): " << f->shell->camToWorld.matrix3x4() << std::endl;
  //       // }
  //       Vec4 pt_world = T_newestF_currF * pt_cam * 20.6128;

  //       pcl::PointXYZRGB pt_w(255, 255, 255);
  //       pt_w.x = pt_world[0];
  //       pt_w.y = pt_world[1];
  //       pt_w.z = pt_world[2];
  //       cloud.push_back(pt_w);
  //       num_pt_marg++;
  //       // std::cout << "pt_w: " << pt_w.x << " " << pt_w.y << " " << pt_w.z << " coming from " << p->u << " " << p->v
  //                 // << " and p->idepth = " << p->idepth << std::endl;
  //     }

  //     for (PointHessian* p : f->pointHessiansOut) {
  //       Vec4 pt_cam;
  //       // TODO: verify that idepth == inverse depth
  //       float depth = 1.0f / p->idepth;
  //       auto const x = (p->u * fxi + cxi) * depth;
  //       auto const y = (p->v * fyi + cyi) * depth;
  //       auto const z = depth * (1 + 2*fxi);
        
  //       pt_cam[0] = x;
  //       pt_cam[1] = y;
  //       pt_cam[2] = z;
  //       pt_cam[3] = 1.f;
  //       Vec4 pt_world = T_newestF_currF * pt_cam * 20.6128;

  //       pcl::PointXYZRGB pt_w(255, 255, 255);
  //       pt_w.x = pt_world[0];
  //       pt_w.y = pt_world[1];
  //       pt_w.z = pt_world[2];
  //       cloud.push_back(pt_w);
  //     }

  //     for (ImmaturePoint* p : f->immaturePoints) {
  //       Vec4 pt_cam;
  //       float idepth = (p->idepth_max + p->idepth_min) * 0.5f;
  //       float depth = 1.0f / idepth;
  //       auto const x = (p->u * fxi + cxi) * depth;
  //       auto const y = (p->v * fyi + cyi) * depth;
  //       auto const z = depth * (1 + 2*fxi);
        
  //       pt_cam[0] = x;
  //       pt_cam[1] = y;
  //       pt_cam[2] = z;
  //       pt_cam[3] = 1.f;
  //       Vec4 pt_world = T_newestF_currF * pt_cam * 20.6128;

  //       pcl::PointXYZRGB pt_w(255, 255, 255);
  //       pt_w.x = pt_world[0];
  //       pt_w.y = pt_world[1];
  //       pt_w.z = pt_world[2];
  //       cloud.push_back(pt_w);
  //     }

  //     // TODO: watch out, f->pointHessians is a normal ptr, there may be some dereferencing issues. Might have to make
  //     // copies instead. for each point, backproject into KF and transform using that KF pose

  //     //   accumulated_pointcloud.insert(accumulated_pointcloud.end(), f->pointHessians.begin(),
  //     //   f->pointHessians.end());
  //   }
    
  //   ROS_WARN("num marginalized points: %d\n", num_pt_marg);

  //   // TODO: extract point cloud near current pose. Maybe use KNN library? PCL?
  //   // std::string current_pose =

  //   sensor_msgs::PointCloud2 ros_pointcloud;
  //   pcl::toROSMsg(cloud, ros_pointcloud);
  //   ros_pointcloud.header.frame_id = "cam02";
  //   // ros_pointcloud.header.frame_id = "world";
  //   // ROS_WARN("pc timestamp: %f\n", ros_pointcloud.header.stamp.toSec());

  //   // get most current frame's timestamp
  //   // double timestamp = frames.front()->shell->timestamp;
  //   // ROS_WARN("pc timestamp front: %f\n", timestamp);
  //   double timestamp = frames.back()->shell->timestamp;
  //   ROS_WARN("pc timestamp back: %f\n", timestamp);
  //   ros::Time time(timestamp);
  //   // ros_pointcloud.header.stamp = ros::Time::now();  // TODO: use time of correct TF
  //   ros_pointcloud.header.stamp = time;  // TODO: use time of correct TF

  //   // loop over all KFs (and accumulate points inside a certain radius?)
  //   std_msgs::String msg;
  //   msg.data = "Number of accumulated points: " + std::to_string(cloud.size()) + "\n";

  //   ROS_WARN("Number of accumulated points: %lu\n", cloud.size());
  //   pointcloud_info_pub.publish(msg);
  //   pointcloud_pub.publish(ros_pointcloud);
  // }

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

            if (final)
            {
                for (FrameHessian* f : frames)
                {
                    if (f->shell->poseValid)
                    {
                        auto const& m = f->shell->camToWorld.matrix3x4();

                        // use only marginalized points.
                        auto const& points = f->pointHessiansMarginalized;

                        for (auto const* p : points)
                        {
                            float depth = 1.0f / p->idepth;
                            auto const x = (p->u * fxi + cxi) * depth;
                            auto const y = (p->v * fyi + cyi) * depth;
                            auto const z = depth * (1 + 2 * fxi);

                            Eigen::Vector4d camPoint(x, y, z, 1.f);
                            Eigen::Vector3d worldPoint = m * camPoint * 20.6128;

                            if (isSavePCL && pclFile.is_open())
                            {
                                isWritePCL = true;

                                pclFile << worldPoint[0] << " " << worldPoint[1] << " " << worldPoint[2] << "\n";

                                // printf("[%d] Point Cloud Coordinate> X: %.2f, Y: %.2f, Z: %.2f\n",
                                //          numPCL,
                                //          worldPoint[0],
                                //          worldPoint[1],
                                //          worldPoint[2]);

                                numPCL++;
                                isWritePCL = false;
                            }
                            else
                            {
                                if (!isPCLfileClose)
                                {
                                    if (pclFile.is_open())
                                    {
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