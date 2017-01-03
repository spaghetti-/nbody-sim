#include "body.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_ptr.hpp>
#include <random>
#include <pcl/point_traits.h>
#include <pcl/common/distances.h>

constexpr double G = 6.67428e-11;

template <typename T>
boost::shared_ptr<pcl::visualization::PCLVisualizer>
get_viewer(typename pcl::PointCloud<T>::ConstPtr cloud, int size)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("viewer")
      );
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<T>(cloud, "samples");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "samples"
      );
  viewer->initCameraParameters();
  return viewer;
}

void disturb(pcl::PointCloud<Body>::Ptr cloud) {
  for (int i = 0; i < cloud->points.size(); i++) {
    for (int j = 0; j < cloud->points.size(); j++) {
      double r = pcl::squaredEuclideanDistance(
          cloud->points[j],
          cloud->points[i]
          ) + 1e2;
      cloud->points[i].force() = 
        G * (
        ((cloud->points[i].m * cloud->points[j].m) / (r)) *
        (cloud->points[j].getVector3fMap() - cloud->points[i].getVector3fMap()).cast<double>());
    }
    Eigen::Vector3d acc = (cloud->points[i].force() / cloud->points[i].m);
    cloud->points[i].velocity() = cloud->points[i].velocity() +
      (acc) * (1/30.0);
    cloud->points[i].getVector3fMap() =
      cloud->points[i].getVector3fMap() +
      (cloud->points[i].velocity().cast<float>() * (1/30.0)) +
      (0.5 * acc.cast<float>() * (1/30.0) * (1/30.0));
  }
}


int main(int argc, char *argv[])
{
  pcl::PointCloud<Body>::Ptr points(new pcl::PointCloud<Body>);
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> dist(1, 1000.0);
  std::uniform_real_distribution<double> mass_dist(1e9, 1e13);

  int nbody = 100;
  int point_size = 2;
  if (argc >= 2) {
    nbody = std::atoi(argv[1]);
  }

  if (nbody < 10) point_size = 4;

  for (int i = 0; i < nbody; i++) {
    points->push_back(Body(dist(mt), dist(mt), dist(mt), mass_dist(mt)));
  }
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = get_viewer<Body>(
      points, point_size);
  while (!viewer->wasStopped()) {
    disturb(points);
    viewer->updatePointCloud<Body>(points, "samples");
    viewer->spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(3));
  }
  return 0;
}
