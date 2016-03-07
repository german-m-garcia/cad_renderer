/*
 * Renderer.h
 *
 *  Created on: 7 Mar 2016
 *      Author: martin
 *
 *  Based on pcl code from:
 *  http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_%28pipeline%29#Getting_partial_views
 */

#ifndef RENDERER_H_
#define RENDERER_H_

#include <pcl/io/pcd_io.h>

class Renderer {
public:
	Renderer(std::string& model, std::string& output_path);
	virtual ~Renderer();

	void
	render_view(double view_x, double view_y, double view_z, double pos_x, double pos_y, double pos_z,double req_yaw , pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& renderization);

	void
	visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

	void
	render_views(double view_x, double view_y,
			double view_z, double pos_x, double pos_y, double pos_z,
			std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& renderizations);

	void render_all_views(std::vector < pcl::PointCloud < pcl::PointXYZRGBA > ::Ptr >& views,
			std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& poses);

private:


	void switch_axes(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

	void switch_axes(double& x, double&y, double& z);

	void add_at_position(Eigen::Vector3f& pos,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object);

	void add_observation_point(Eigen::Vector3f& observation,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

	void create_frame(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

	void add_line_to_target(Eigen::Vector3f&dest,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

	void undo_yaw_rotation(double& yaw,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

	void simulate_yaw_rotation(double yaw, Eigen::Vector3f& cam_values);

	void get_cam_values(Eigen::Vector3f& origin, Eigen::Vector3f& dest, Eigen::Vector3f& cam_values);


	std::string model_path,output_path;


};

#endif /* RENDERER_H_ */
