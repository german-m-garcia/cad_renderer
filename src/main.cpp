/*
 * main.cpp
 *
 *  Created on: 7 Mar 2016
 *      Author: martin
 */

#include "Renderer.h"
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataMapper.h>
#include "render_views_tesselated_sphere_rgb.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>


int
main(int argc, char** argv)
{
	if(argc != 8){
		std::cout <<"Usage: ./snapshotCreatorRGB <path to CAD model> <obj X> <obj Y> <obj Z> <view X> <view X> <view X>"<<std::endl;
		return -1;
	}

	std::string model_path(argv[1]);
	Renderer renderer(model_path);
	double pos_x = atoi(argv[2]);
	double pos_y = atoi(argv[3]);
	double pos_z = atoi(argv[4]);
	double view_x = atoi(argv[5]);
	double view_y = atoi(argv[6]);
	double view_z = atoi(argv[7]);
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> renderizations;
	renderer.render_views(view_x,view_y, view_z, pos_x, pos_y, pos_z, renderizations);

	for(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud: renderizations){
		renderer.visualize(cloud);
	}


	//to test the renderization of just one possible yaw, run the following:
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr renderization;
	//double yaw = 0.;
	//renderer.render_view(view_x,view_y, view_z, pos_x, pos_y, pos_z, yaw, renderization);

	//for (double yaw = 0.0; yaw < 0.45; yaw += 0.05){
	//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr renderization;
	//	renderer.render_view(view_x,view_y, view_z, pos_x, pos_y, pos_z, yaw, renderization);
	//	renderer.visualize(renderization);
	//}

	return 0;

}
