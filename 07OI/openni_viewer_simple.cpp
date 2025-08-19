#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>


#include <chrono>
#include <thread>

using namespace std::chrono_literals;


class SimpleOpenNIViewer
{
public:
  SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
  {
    if (!viewer.wasStopped())
      viewer.showCloud (cloud);
  }

  void run ()
  {
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    std::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        [this] (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) { cloud_cb_ (cloud); };

    interface->registerCallback (f);

    interface->start ();

    while (!viewer.wasStopped())
    {
      std::this_thread::sleep_for(1s);
    }

    interface->stop ();
  }

  pcl::visualization::CloudViewer viewer;
};

int main ()
{
  SimpleOpenNIViewer v;
  v.run ();
  return 0;
}