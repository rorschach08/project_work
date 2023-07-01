// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>
#include <random>

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliers;
	std::vector<std::unordered_set<int>> inliersPerIteration;

	srand(time(NULL));


	unsigned seed = 0;
	int maxInliers = 0;
	int count=0;
	
	int minPointsForLine = 2;
	int maxInliersIndex = 0;

	std::vector<std::pair<float, float>> twoPoints;

	std::vector<int> indices;
	
	for(auto k = 0; k < cloud->points.size(); k++)
	{
		indices.push_back(k);
	}
	
	shuffle(indices.begin(), indices.end(), std::default_random_engine(seed));


	// TO: Fill in this function

	// For max iterations 
	
	while(maxIterations)
	{
		
	// Randomly sample subset and fit line
		twoPoints.clear();
		int j = count;
		for(int l = 0; l < minPointsForLine; l++)
		{
			twoPoints.push_back(std::make_pair(cloud->points[indices[j]].x, cloud->points[indices[j]].y));
			j++;
		}
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
		for(int index = 0; index < cloud->points.size(); index++)
		{
			pcl::PointXYZ point = cloud->points[index];
			
			float distanceToLine = abs((twoPoints[1].first-twoPoints[0].first)*(point.y-twoPoints[0].second)-(twoPoints[1].second-twoPoints[0].second)*(point.x-twoPoints[0].first))/sqrt(pow((twoPoints[1].first-twoPoints[0].first),2)+pow((twoPoints[1].second-twoPoints[0].second),2));

			if(distanceToLine < distanceTol)
			{inliers.insert(index);}
		}

		if (inliers.size() > maxInliers)
		{
			maxInliers = inliers.size();
			inliersResult.clear();
			inliersResult.insert(inliers.begin(), inliers.end());
		}
		

		inliers.clear();
		//inliersPerIteration.push_back(inliers);
		maxIterations--;
		count++;

	} 
	// Return indicies of inliers from fitted line with most inliers
	
	
	/*
	for(auto itr = 0; itr < inliersPerIteration.size(); itr++)
	{
		if (inliersPerIteration[itr].size() > maxInliers)
		{
			maxInliers = inliersPerIteration[itr].size();
			maxInliersIndex = itr;
		}
		
	}

	inliersResult.insert(inliersPerIteration[maxInliersIndex].begin(), inliersPerIteration[maxInliersIndex].end());
	*/

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.1);

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
