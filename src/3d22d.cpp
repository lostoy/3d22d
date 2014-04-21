#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/octree/octree.h>

#include <iostream>
#include <fstream>

#include <set>

void loadconfig(std::string configfile,std::string &plyinpath,std::string &plyoutpath,float &radius,int &minNum)
{
	std::fstream file(configfile, std::ios::in);
	file >> plyinpath;
	file >> plyoutpath;
	file >> radius;
	file >> minNum;
}
int main()
{
	std::string plyinpath,plyoutpath;
	float radius;
	int minNum;
	loadconfig("../3d22d_config.txt", plyinpath,plyoutpath ,radius,minNum);

	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	pcl::io::loadPLYFile(plyinpath, cloud);

	int total_points = cloud.size();
	std::cout << "total points: " << total_points << std::endl;
	float progress = 0;
	for (int i = 0; i < cloud.size(); i++)
	{
		cloud.points[i].y = 0;
		if (i*1.0 / total_points>progress + 0.05)
		{
			progress = i*1.0 / total_points;
			std::cout << progress << std::endl;
		}
	}
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> octree(radius);
	octree.setInputCloud(cloud.makeShared());
	octree.addPointsFromInputCloud();

	pcl::PointCloud<pcl::PointXYZRGBA> cloud_out;

	std::set<int> ignores;
	for (int i = 0; i<total_points; i++)
	{
		if (ignores.count(i)>0)
			continue;

		std::vector<int> inliers;
		octree.voxelSearch(i, inliers);
		if (inliers.size()>minNum)
		{
			cloud_out.push_back(cloud.points[i]);
			for (int j = 0; j < inliers.size(); j++)
				ignores.insert(inliers[j]);
		}

	}
	
	pcl::io::savePLYFileBinary(plyoutpath, cloud_out);

}

