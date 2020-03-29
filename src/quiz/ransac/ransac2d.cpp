/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	for(int iter = 0; iter < maxIterations; iter++)
	{
		std::unordered_set<int> inliers;

		// Randomly sample subset and fit line
		int index1 = rand() % (cloud->points.size());
		int index2 = rand() % (cloud->points.size());
		
		float x1 = cloud->points[index1].x;
		float y1 = cloud->points[index1].y;
		float x2 = cloud->points[index2].x;
		float y2 = cloud->points[index2].y;

		float a = y2 - y1;
		float b = x2 - x1;
		float c = x1 * y2 - y1 * x2;
	
		// Measure distance between every point and fitted line
		double divisor = sqrt(a * a + b * b);

		if(divisor == 0)
			continue;

		for(int index = 0; index < cloud->points.size(); index++)
		{
			float x3 = cloud->points[index].x;
			float y3 = cloud->points[index].y;

			float distance = fabs(a * x3 + b * y3 + c) / divisor;

			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol)
				inliers.insert(index);
		}
		
		// Return indicies of inliers from fitted line with most inliers
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	return inliersResult;
}

std::unordered_set<int> Ransac3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	float epsilon = 0.001f;

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	for(int iter = 0; iter < maxIterations; iter++)
	{
		std::unordered_set<int> inliers;

		// Randomly sample subset and fit line
		int index1 = rand() % (cloud->points.size());
		int index2 = rand() % (cloud->points.size());
		int index3 = rand() % (cloud->points.size());

		//std::cout << index1 << " " << index2 << " " << index3 << std::endl;

		float x1 = cloud->points[index1].x;
		float y1 = cloud->points[index1].y;
		float z1 = cloud->points[index1].z;
		float x2 = cloud->points[index2].x;
		float y2 = cloud->points[index2].y;
		float z2 = cloud->points[index2].z;
		float x3 = cloud->points[index3].x;
		float y3 = cloud->points[index3].y;
		float z3 = cloud->points[index3].z;

		//Vector normal to the v1 and v2
		float normx = (y2-y1) * (z3-z1) - (z2-z1) * (y3-y1); 
		float normy = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
		float normz = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);

		//Equation of the plane given the 3 points: Ax + Bx + Cx + D = 0
		float a = normx;
		float b = normy;
		float c = normz;
		float d = -(normx* x1 + normy * y1 + normz * z1) ;
	
		// Measure distance between every point and fitted line
		float divisor = sqrt(a * a + b * b + c * c);
		
		if(divisor <= std::numeric_limits<float>::epsilon()) // epsilon)
			continue;

		for(int index = 0; index < cloud->points.size(); index++)
		{
			float x3 = cloud->points[index].x;
			float y3 = cloud->points[index].y;
			float z3 = cloud->points[index].z;

			float distance = fabs(a * x3 + b * y3 + c * z3 + d) / divisor;

			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol)
				inliers.insert(index);
		}
		
		//std::cout << inliers.size() << " > " << inliersResult.size() << std::endl;

		// Return indicies of inliers from fitted line with most inliers
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	return inliersResult;
}

int main (int argc, char** argv)
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	int maxIterations = 50;
	float tolerance = 0.1;

	if(argc >= 3)
	{
		maxIterations = std::stoi(argv[1]);
		tolerance = std::atof(argv[2]);
	}
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac2d(cloud, maxIterations, tolerance);
	std::unordered_set<int> inliers = Ransac3d(cloud, maxIterations, tolerance);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
