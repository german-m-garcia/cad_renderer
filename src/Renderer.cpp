/*
 *  Renderer.cpp
 *
 *  Created on: 7 Mar 2016
 *      Author: martin
 *  Based on pcl code from:
 *  http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_%28pipeline%29#Getting_partial_views
 *
 *
 */

#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataMapper.h>
#include "render_views_tesselated_sphere_rgb.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include "Renderer.h"

Renderer::Renderer(std::string& model,std::string& output_path): model_path(model),
	output_path(output_path){

}

Renderer::~Renderer() {

}

void Renderer::get_cam_values(Eigen::Vector3f& origin, Eigen::Vector3f& dest,
		Eigen::Vector3f& cam_values) {

	cam_values[0] = origin[0] - dest[0];
	cam_values[1] = origin[1] - dest[1];
	cam_values[2] = origin[2] - dest[2];
	cam_values.normalize();
	//std::cout << "get_cam_values::norm of cam_values=" << cam_values.norm()
	//		<< std::endl;
}

void Renderer::simulate_yaw_rotation(double yaw, Eigen::Vector3f& cam_values) {
	//the rotation is actually around the Y axis which is the vertical one in the CAD model
	double cur_x = cam_values[0];
	//double radius = 1-fabs(cam_values[1]);
	double h = 1 - cam_values[1];
	//https://en.wikipedia.org/wiki/Spherical_cap
	double radius = sqrt(2 * h - h * h);
	double cur_z = cam_values[2];
	double current_yaw = atan2(cur_x, cur_z);
	//std::cout << "> current yaw=" << current_yaw << std::endl;

	yaw += current_yaw;
	//std::cout << "> requested yaw=" << yaw << std::endl;
	cam_values[0] = sin(yaw) * radius;
	cam_values[2] = cos(yaw) * radius;
	//std::cout << "simulate_yaw_rotation::yaw trick cam norm="
	//		<< cam_values.norm() << std::endl;
}

void Renderer::undo_yaw_rotation(double& yaw,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	// The same rotation matrix as before; tetha radians arround Y axis
	//std::cout << "> undo_yaw_rotation yaw=" << -yaw << std::endl;
	transform.rotate(Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitY()));

	pcl::transformPointCloud(*cloud, *cloud, transform);

}

void Renderer::add_line_to_target(Eigen::Vector3f&dest,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

	double x = dest[0];
	double z = dest[2];
	double alpha = atan2(z, x);
	double mod = sqrt(x * x + z * z);
	for (double length = 0.; length < mod; length += 1.) {

		double x_ = length * cos(alpha);
		double z_ = length * sin(alpha);
		pcl::PointXYZRGBA point;
		point.x = x_;
		point.y = 0;
		point.z = z_;
		cloud->push_back(point);

	}
	/*
	 for(double x = 0., y = 0.; x < dest[0]; x+= 1.){
	 pcl::PointXYZRGBA point;
	 point.x = x;
	 point.y = 0;
	 point.z = x;
	 cloud->push_back(point);
	 }*/

}

void Renderer::create_frame(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

	for (double x = 0.; x < 200.; x += 1.) {
		pcl::PointXYZRGBA point;
		point.x = x;
		point.y = 0.;
		point.z = 0.;
		cloud->push_back(point);

		pcl::PointXYZRGBA point2;
		point2.x = 0.;
		point2.y = x;
		point2.z = 0.;
		cloud->push_back(point2);

		pcl::PointXYZRGBA point3;
		point3.x = 0.;
		point3.y = 0.;
		point3.z = x;
		cloud->push_back(point3);
	}

}

void Renderer::add_observation_point(Eigen::Vector3f& observation,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

	pcl::PointXYZRGBA obs;
	obs.x = observation[0];
	obs.y = observation[1];
	obs.z = observation[2];
	obs.g = 0;
	obs.b = 0;
	obs.r = 255;
	cloud->push_back(obs);
}

void Renderer::add_at_position(Eigen::Vector3f& pos,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object) {

	double shift_x = pos[0];
	double shift_y = pos[1];
	double shift_z = pos[2];

	for (pcl::PointXYZRGBA& obj_point : *object) {
		obj_point.x += shift_x;
		obj_point.y += shift_y;
		obj_point.z += shift_z;
		cloud->push_back(obj_point);
	}

}

void Renderer::switch_axes(double& x, double&y, double& z) {
	double x_ = x;
	double y_ = y;
	double z_ = z;
	x = y_;
	z = x_;
	y = z_;
}

void Renderer::switch_axes(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {

	for (pcl::PointXYZRGBA& p : *cloud) {
		double x = p.x;
		double y = p.y;
		double z = p.z;
		p.x = z;
		p.y = x;
		p.z = y;
	}
}

void Renderer::visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
	pcl::visualization::PCLVisualizer viewer("CLOUD");
	viewer.addPointCloud < pcl::PointXYZRGBA > (cloud, "CLOUD");
	viewer.addCoordinateSystem(100.5);
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "CLOUD");
	viewer.spin();
}

void Renderer::render_view(double view_x, double view_y,
		double view_z, double pos_x, double pos_y, double pos_z, double req_yaw,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& renderization) {
	// Load the PLY model from a file.
	vtkSmartPointer < vtkPLYReader > reader = vtkSmartPointer < vtkPLYReader
			> ::New();
	reader->SetFileName(model_path.c_str());
	reader->Update();

	// VTK is not exactly straightforward...
	vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer
			< vtkPolyDataMapper > ::New();
	mapper->SetInputConnection(reader->GetOutputPort());
	mapper->Update();

	vtkSmartPointer < vtkPolyData > object = mapper->GetInput();

	// Virtual scanner object.
	pcl::apps::RenderViewsTesselatedSphereRGBA render_views;
	render_views.addModelFromPolyData(object);
	// Pixel width of the rendering window, it directly affects the snapshot file size.
	render_views.setResolution(350);
	// Horizontal FoV of the virtual camera.
	render_views.setViewAngle(57.0f);
	// If true, the resulting clouds of the snapshots will be organized.
	render_views.setGenOrganized(true);
	// How much to subdivide the icosahedron. Increasing this will result in a lot more snapshots.
	render_views.setTesselationLevel(1);
	// If true, the camera will be placed at the vertices of the triangles. If false, at the centers.
	// This will affect the number of snapshots produced (if true, less will be made).
	// True: 42 for level 1, 162 for level 2, 642 for level 3...
	// False: 80 for level 1, 320 for level 2, 1280 for level 3...
	render_views.setUseVertices(true);
	// If true, the entropies (the amount of occlusions) will be computed for each snapshot (optional).
	render_views.setComputeEntropies(true);

	//render_views.generateViews();
	//generate a single view instead
	//							    X   Y   Z	
	Eigen::Vector3f cam_position(1.01, 1.0, 0.5);
	cam_position = cam_position.normalized();

	switch_axes(pos_x, pos_y, pos_z);

	double orig_x = view_x;	//atoi(argv[5]);
	double orig_y = view_y;	//atoi(argv[6]);
	double orig_z = view_z;	//atoi(argv[7]);
	switch_axes(orig_x, orig_y, orig_z);
	Eigen::Vector3f origin(orig_x, orig_y, orig_z);
	Eigen::Vector3f dest(pos_x, pos_y, pos_z);

	get_cam_values(origin, dest, cam_position);
	//std::cout << "cam_position=" << cam_position[0] << " " << cam_position[1]
	//		<< " " << cam_position[2] << std::endl;

	//iterate for a number of rotations of the model

	//defines a rotation around the (Y) axis of the original model		
	simulate_yaw_rotation(req_yaw, cam_position);
	//std::cout << "cam_position with req_yaw=" << cam_position[0] << " "
	//		<< cam_position[1] << " " << cam_position[2] << std::endl;
	render_views.generateView(cam_position);

	// Object for storing the rendered views.
	std::vector < pcl::PointCloud < pcl::PointXYZRGBA > ::Ptr > views;
	// Object for storing the poses, as 4x4 transformation matrices.
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
	// Object for storing the entropies (optional).
	std::vector<float> entropies;
	render_views.getViews(views);
	render_views.getPoses(poses);
	render_views.getEntropies(entropies);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr frame(
			new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Save all views to disk.
	bool save_views = false;
	if(save_views)
	for (size_t i = 0; i < views.size(); i++) {
		std::stringstream stream;
		stream << "view" << i << ".pcd";
		//visualize it
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
				new pcl::PointCloud<pcl::PointXYZRGBA>);
		cloud = views.at(i);
		std::string filename = output_path+stream.str();
		std::cout <<" saving at "<<filename<<std::endl;

		if (pcl::io::savePCDFile(filename, *(views.at(i)), true) != 0)
			PCL_ERROR("Problem saving %s.\n", filename.c_str());



		Eigen::Matrix4f inverse_pose = poses.at(i).inverse();

		pcl::transformPointCloud(*cloud, *cloud, inverse_pose);

		//undo the requested yaw rotation
		undo_yaw_rotation(req_yaw, cloud);


		//create a dummy 3D world
		//create_frame(frame);	
		//Eigen::Vector3f dest2(dest[0]*10,dest[0]*10,dest[0]*10);		
		add_at_position(dest, frame, cloud);

		//add_observation_point(origin, cloud);

		//add_line_to_target(dest, cloud);

		switch_axes(cloud);

		renderization = cloud;

	}
}

/*
 * for a given view point places the object at the specified (expected) position,
 * and renders several views by rotating the model around its vertical axis (Z)
 *
 */
void Renderer::render_views(double view_x, double view_y,
		double view_z, double pos_x, double pos_y, double pos_z,
		std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& renderizations){

	for (double yaw = 0.0; yaw < 1.58; yaw += 0.05){
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr renderization;
		render_view(view_x,view_y, view_z, pos_x, pos_y, pos_z, yaw, renderization);
		renderizations.push_back(renderization);
	}
}

/*
 * renders all possible views around the object
 *
 * 	@views: Object for storing the rendered views
 * 	@poses: Object for storing the poses, as 4x4 transformation matrices.
 *
 */
void Renderer::render_all_views(std::vector < pcl::PointCloud < pcl::PointXYZRGBA > ::Ptr >& views,
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& poses) {


	// Load the PLY model from a file.
	vtkSmartPointer < vtkPLYReader > reader = vtkSmartPointer < vtkPLYReader
			> ::New();
	reader->SetFileName(model_path.c_str());
	reader->Update();

	// VTK is not exactly straightforward...
	vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer
			< vtkPolyDataMapper > ::New();
	mapper->SetInputConnection(reader->GetOutputPort());
	mapper->Update();

	vtkSmartPointer < vtkPolyData > object = mapper->GetInput();

	// Virtual scanner object.
	pcl::apps::RenderViewsTesselatedSphereRGBA render_views;
	render_views.addModelFromPolyData(object);
	// Pixel width of the rendering window, it directly affects the snapshot file size.
	render_views.setResolution(350);
	// Horizontal FoV of the virtual camera.
	render_views.setViewAngle(57.0f);
	// If true, the resulting clouds of the snapshots will be organized.
	render_views.setGenOrganized(true);
	// How much to subdivide the icosahedron. Increasing this will result in a lot more snapshots.
	render_views.setTesselationLevel(1);
	// If true, the camera will be placed at the vertices of the triangles. If false, at the centers.
	// This will affect the number of snapshots produced (if true, less will be made).
	// True: 42 for level 1, 162 for level 2, 642 for level 3...
	// False: 80 for level 1, 320 for level 2, 1280 for level 3...
	render_views.setUseVertices(true);
	// If true, the entropies (the amount of occlusions) will be computed for each snapshot (optional).
	render_views.setComputeEntropies(true);
	render_views.generateViews();

	// Object for storing the entropies (optional).
	std::vector<float> entropies;
	render_views.getViews(views);
	render_views.getPoses(poses);
	render_views.getEntropies(entropies);

	// Save all views to disk.
	for (size_t i = 0; i < views.size(); i++) {
		std::stringstream stream;
		stream << "view" << i << ".pcd";
		std::string filename = output_path+stream.str();

		//visualize it
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
				new pcl::PointCloud<pcl::PointXYZRGBA>);
		cloud = views.at(i);


		//Eigen::Matrix4f inverse_pose = poses.at(i).inverse();
		//pcl::transformPointCloud(*cloud, *cloud, inverse_pose);

		switch_axes(cloud);
		if (pcl::io::savePCDFile(filename, *cloud, true) != 0)
			PCL_ERROR("Problem saving %s.\n", filename.c_str());

	}
}
