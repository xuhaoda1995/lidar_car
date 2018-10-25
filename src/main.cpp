#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/io/grabber.h>
#include <pcl/console/parse.h>

//#include<pcl/range_image/range_image.h>

#include "hazard_point_detect.h"
#include "grid_obs_create.h"
#include "cloud_publisher.h"
#include "hdl_grabber.h"

ros::Publisher pub;
using namespace pcl;
using namespace pcl::console;

int g_frame_num = 0;

#define SHOW_FPS 0
#if SHOW_FPS
#define FPS_CALC(_WHAT_)                                                                                                      \
	do                                                                                                                        \
	{                                                                                                                         \
		static unsigned count = 0;                                                                                            \
		static double last = getTime();                                                                                       \
		double now = getTime();                                                                                               \
		++count;                                                                                                              \
		if (now - last >= 1.0)                                                                                                \
		{                                                                                                                     \
			std::cout << "Average framerate(" << _WHAT_ << "): " << double(count) / double(now - last) << " Hz" << std::endl; \
			count = 0;                                                                                                        \
			last = now;                                                                                                       \
		}                                                                                                                     \
	} while (false)
#else
#define FPS_CALC(_WHAT_) \
	do                   \
	{                    \
	} while (false)
#endif

///////////////////////////////////////////////////////////////////////////////
#define MAX_CLOUD_SISE 70000

typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr CloudConstPtr;
typedef pcl::PointCloud<pcl::PointSrc>::ConstPtr CloudSrcPtr;

class HDL32EViewer
{
  public:
	HDL32EViewer(Grabber &grabber)
		: grabber_(grabber)
	{
	}

	void
	cloud_callback(const CloudConstPtr &cloud, const CloudSrcPtr &cloudsrc)
	{
		FPS_CALC("cloud callback");
		boost::mutex::scoped_lock lock(cloud_mutex_);
		cloud_ = cloud;
		cloudsrc_ = cloudsrc;
	}

	void
	cloud_callback(const CloudConstPtr &cloud, float startAngle,
				   float endAngle)
	{
		FPS_CALC("cloud callback");
		boost::mutex::scoped_lock lock(cloud_mutex_);
		cloud_ = cloud;
	}

	void initial()
	{
		//calculate obstacle _angle threshold, run only once
		filterThresholdOfObstacle(g_LiDAR_pos[2], g_LiDAR_pos[3], 0, 0, 0.1);

		//initial publisher
		cloud_show::init_pub();
	}

	void
	run()
	{
		initial();

		///////////////////////////////////////////////////////////////////////////////
		//open pcap grabber thread
		boost::function<void(const CloudConstPtr &, const CloudSrcPtr &)> cloud_cb = boost::bind(

			&HDL32EViewer::cloud_callback, this, _1, _2);
		boost::signals2::connection cloud_connection = grabber_.registerCallback(
			cloud_cb);

		grabber_.start();
		///////////////////////////////////////////////////////////////////////////////

		int frame_num = 1;
		while (true)
		{
			CloudConstPtr cloud;
			CloudSrcPtr cloudsrc;
			// See if we can get a cloud
			if (cloud_mutex_.try_lock())
			{
				cloud_.swap(cloud);
				cloudsrc_.swap(cloudsrc);
				cloud_mutex_.unlock();
			}
			if (cloud)
			{
				double t1 = pcl::getTime();

				///////////////////////////////////////////////////////////////////////////////
				//cloud pretreatment,get cloud taking vehicle as coordinate center
				pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZI>);
				for(auto pt : cloud->points)
				{
					pt=transform_point(pt);
					cloud_ori->push_back(pt);
				}
				///////////////////////////////////////////////////////////////////////////////

				///////////////////////////////////////////////////////////////////////////////
				//obstacle detection
				vector<int> obs_idx;
				HazardDetection hazardDetection;
				hazardDetection.detectHazardPoint(cloud, cloudsrc, obs_idx);
				std::cout << "obs time is " << pcl::getTime() - t1 << "s" << std::endl;
				///////////////////////////////////////////////////////////////////////////////

				///////////////////////////////////////////////////////////////////////////////
				//rasterize
				GridCreator grid_obs;
				grid_obs.createGrid(cloud, obs_idx);
				///////////////////////////////////////////////////////////////////////////////

				pcl::PointCloud<pcl::PointXYZI>::Ptr obsCloud(new pcl::PointCloud<pcl::PointXYZI>);
				for (auto idx : obs_idx)
				{
					obsCloud->push_back(cloud_ori->points[idx]);
				}

				//publish result
				//TODO:need change according to other message required
				cloud_show::all_publish(obsCloud, cloud_ori);

				g_frame_num++;

				std::cout << "total time is " << pcl::getTime() - t1 << std::endl;
			}
		}

		grabber_.stop();

		cloud_connection.disconnect();
	}

	time_t velo_gpstime_;
	Grabber &grabber_;
	boost::mutex cloud_mutex_;

	CloudConstPtr cloud_;
	CloudSrcPtr cloudsrc_;
};

void usage(char **argv)
{
	cout << "usage: " << argv[0]
		 << " [-hdlCalibration <path-to-calibration-file>] [-pcapFile <path-to-pcap-file>] [-h | --help] [-format XYZ(default)|XYZI|XYZRGB]"
		 << endl;
	cout << argv[0] << " -h | --help : shows this help" << endl;
	return;
}

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "velo_process");
	// ros::NodeHandle nh;

	std::string hdlCalibration, pcapFile, format("XYZI");

	if (find_switch(argc, argv, "-h") ||
		find_switch(argc, argv, "--help"))
	{
		usage(argv);
		return (0);
	}

	parse_argument(argc, argv, "-calibrationFile", hdlCalibration);
	parse_argument(argc, argv, "-pcapFile", pcapFile);
	parse_argument(argc, argv, "-format", format);

	HDLGrabber grabber(hdlCalibration, pcapFile);

	cout << "viewer format:" << format << endl;

	if (boost::iequals(format, std::string("XYZI")))
	{
		HDL32EViewer v(grabber);
		v.run();
	}

	return (0);
}