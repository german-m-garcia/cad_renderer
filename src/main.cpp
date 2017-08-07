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

	if(argc == 3){

		std::string model_path(argv[1]);
		std::string output_path(argv[2]);
		Renderer renderer(model_path,output_path);
		/*
		 * ---------------------------------------------------------------------
		 * to test the renderization of all possible view points, run the following:
		 *
		 */
		std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> renderizations;
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
		renderer.render_all_views( renderizations,poses);

		std::ofstream file("test.txt");
		int index = 0;
		for(Eigen::Matrix4f& pose : poses)
		{

		  if (file.is_open())
		  {

			file << "View"<<index<<".pcd<<\n" << pose << '\n';
			index++;
			//file << "m" << '\n' <<  colm(pose) << '\n';
		  }
		}
		return 0;

	}

	if(argc != 8){
		std::cout <<"Usage: ./snapshotCreatorRGB <path to CAD model> <obj X> <obj Y> <obj Z> <view X> <view X> <view X>"<<std::endl;
		return -1;
	}

	std::string model_path(argv[1]);
	std::string output_path("/home/martin/renderizations/");
	Renderer renderer(model_path,output_path);
	double pos_x = atoi(argv[2]);
	double pos_y = atoi(argv[3]);
	double pos_z = atoi(argv[4]);
	double view_x = atoi(argv[5]);
	double view_y = atoi(argv[6]);
	double view_z = atoi(argv[7]);

	/*
	 * ---------------------------------------------------------------------
	 * to test the renderization of just one possible yaw, run the following:
	 *
	 */
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr renderization;
	double yaw = 0.;
	renderer.render_view(view_x,view_y, view_z, pos_x, pos_y, pos_z, yaw, renderization);
	renderer.visualize(renderization);
	std::cout <<"renderization->width="<<renderization->width<<"renderization->height="<<renderization->height<<std::endl;





	/*
	 * ---------------------------------------------------------------------
	 * to test the renderization of all possible yaws, run the following:
	 *
	 */
//	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> renderizations;
//	renderer.render_views(view_x,view_y, view_z, pos_x, pos_y, pos_z, renderizations);
//
//	for(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud: renderizations){
//		renderer.visualize(cloud);
//	}





	return 0;

}
