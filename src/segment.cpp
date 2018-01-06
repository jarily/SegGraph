#include "SegGraph.h"

std::vector<float> clusterFeaExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::vector <float> fea_cluster;
	Eigen::Vector4f centroid;

	pcl::compute3DCentroid(*cloud, centroid);

	fea_cluster.push_back(centroid[0]);
	fea_cluster.push_back(centroid[1]);
	fea_cluster.push_back(centroid[2]);

	float cloud_radius = 0.0;
	float tmp;
	for (int j = 0; j < cloud->points.size(); j++)
	{
		tmp = sqrt((cloud->points[j].x - centroid[0])*(cloud->points[j].x - centroid[0]) +
			(cloud->points[j].y - centroid[1])*(cloud->points[j].y - centroid[1]) +
			(cloud->points[j].z - centroid[2])*(cloud->points[j].z - centroid[2]));

		if (cloud_radius < tmp)
			cloud_radius = tmp;
	}
	//std::cout << "cloud_radius : " << cloud_radius << std::endl;

	//设置搜索的方式或者说是结构
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	//求法线
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	float ave_normal_x = 0.0;
	float ave_normal_y = 0.0;
	float ave_normal_z = 0.0;
	for (int j = 0; j < normals->points.size(); j++)
	{
		ave_normal_x += normals->points[j].normal[0];
		ave_normal_y += normals->points[j].normal[1];
		ave_normal_z += normals->points[j].normal[2];
	}
	ave_normal_x = ave_normal_x / normals->points.size();
	ave_normal_y = ave_normal_y / normals->points.size();
	ave_normal_z = ave_normal_z / normals->points.size();


	fea_cluster.push_back(ave_normal_x);
	fea_cluster.push_back(ave_normal_y);
	fea_cluster.push_back(ave_normal_z);
	fea_cluster.push_back(cloud_radius);
	fea_cluster.push_back(float(cloud->points.size()));
	return fea_cluster;
}


void
segment(char *pcdFile, char *writeFile)
{
	// 读取文件
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr add_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read(pcdFile, *cloud1);
	std::cout << "PointCloud before filtering has: " << cloud1->points.size() << " data points." << std::endl; //*

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_filtered = cloud1;
	//创建平面模型分割的对象并设置参数
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //设置聚类的内点索引
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//平面模型的因子
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);    //分割模型
	seg.setMethodType(pcl::SAC_RANSAC);       //随机参数估计方法
	seg.setMaxIterations(100);                //最大的迭代的次数
	seg.setDistanceThreshold(0.8);           //设置阀值

	int i = 0, nr_points = (int)cloud_filtered->points.size();//剩余点云的数量
	//std::cout << "nr_points=" << nr_points << std::endl;
	//while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) //如果内点的数量已经等于0，就说明没有
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			//break;
		}

		// 从输入的点云中提取平面模型的内点
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);        //提取内点的索引并存储在其中
		extract.setNegative(false);

		// 得到与平面表面相关联的点云数据
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		//  // 移去平面局内点，提取剩余点云
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
	}

	/*
	pcl::visualization::CloudViewer viewer2("移除大地平面后");
	viewer2.showCloud(cloud_filtered);
	while (!viewer2.wasStopped())
	{
	}*/


	//点云的类型
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud = cloud_filtered;
	//设置搜索的方式或者说是结构
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	//求法线
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);
	//直通滤波在Z轴的0到1米之间
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);
	//聚类对象<点，法线>
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(100);  //最小的聚类的点数
	reg.setMaxClusterSize(100000);  //最大的
	reg.setSearchMethod(tree);    //搜索方式
	reg.setNumberOfNeighbours(30);    //设置搜索的邻域点的个数
	reg.setInputCloud(cloud);         //输入点
									  //reg.setIndices (indices);
	reg.setInputNormals(normals);     //输入的法线
	reg.setSmoothnessThreshold(4.0 / 180.0 * M_PI);  //设置平滑度
	reg.setCurvatureThreshold(1.0);     //设置曲率的阀值

	std::vector <pcl::PointIndices> clusters_indices;
	reg.extract(clusters_indices);

	//pcl::PCDWriter writer;
	int j = 0;
	std::vector <std::vector<float> > fea_vector;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters_indices.begin(); it != clusters_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;


		std::vector <float> fea_cluster = clusterFeaExtract(cloud_cluster);
		fea_vector.push_back(fea_cluster);
		fea_cluster.clear();

		j++;
	}

	std::ofstream f;
	f.open(writeFile);

	for (int i = 0; i < fea_vector.size(); i++)
	{
		f << i << " ";
		for (int j = 0; j < fea_vector[i].size(); j++)
		{
			f << fea_vector[i][j] << " ";
		}
		f << std::endl;
	}
	f << std::endl;
	f.close();
	fea_vector.clear();

	//return fea_vector;
	/*
	//pcl::PCDWriter writer;

	std::cout << "Number of clusters is equal to " << clusters_indices.size() << std::endl;


	//可视化聚类的结果
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	//pcl::visualization::CloudViewer viewer5("Cluster viewer");
	//viewer5.showCloud(colored_cloud);
	//while (!viewer5.wasStopped())
	{
	}

	viaClusters(writeFile,j);

	return (j);
	*/
}
