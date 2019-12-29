// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_OPENGL_GLFW_VIEWER_H
#define IGL_OPENGL_GLFW_VIEWER_H

#ifndef IGL_OPENGL_4
#define IGL_OPENGL_4
#endif

#include "../../igl_inline.h"
#include "../MeshGL.h"
//#include "../ViewerCore.h"
#include "../ViewerData.h"
#include "ViewerPlugin.h"
#include "igl/read_triangle_mesh.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <cstdint>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/edge_flaps.h>
#include <my_collapse.h>

#define IGL_MOD_SHIFT           0x0001
#define IGL_MOD_CONTROL         0x0002
#define IGL_MOD_ALT             0x0004
#define IGL_MOD_SUPER           0x0008



namespace igl
{
namespace opengl
{
namespace glfw
{
  // GLFW-based mesh viewer
  class Viewer : public Movable
  {
  public:
    // UI Enumerations
   // enum class MouseButton {Left, Middle, Right};
   // enum class MouseMode { None, Rotation, Zoom, Pan, Translation} mouse_mode;
    IGL_INLINE void init();
    IGL_INLINE void init_plugins();
    IGL_INLINE void shutdown_plugins();
    Viewer();
    ~Viewer();
    // Mesh IO

	void sys_restart();
	int sys_init();
	int load_meshs_ik();
	int load_meshs();
	IGL_INLINE bool init_ds();
	IGL_INLINE bool collapse_edges(int num);
	IGL_INLINE bool my_collapse_edges(int num);
	IGL_INLINE bool collapse_5_percent(int n);
	IGL_INLINE bool quadric_error_handler();
	IGL_INLINE void quadric_error_edge(
		const int e,
		const Eigen::MatrixXd& V,
		const Eigen::MatrixXi& /*F*/,
		const Eigen::MatrixXi& E,
		const Eigen::VectorXi& /*EMAP*/,
		const Eigen::MatrixXi& /*EF*/,
		const Eigen::MatrixXi& /*EI*/,
		double& cost,
		Eigen::RowVectorXd& p);
		IGL_INLINE void quadric_error(
			const int e,
			const Eigen::MatrixXd& V,
			const Eigen::MatrixXi& /*F*/,
			const Eigen::MatrixXi& E,
			const Eigen::VectorXi& /*EMAP*/,
			const Eigen::MatrixXi& /*EF*/,
			const Eigen::MatrixXi& /*EI*/,
			double& cost,
			Eigen::RowVectorXd& p);
    IGL_INLINE bool load_mesh_from_file(const std::string & mesh_file_name);
    IGL_INLINE bool save_mesh_to_file(const std::string & mesh_file_name);
   
    // Scene IO
    IGL_INLINE bool load_scene();
    IGL_INLINE bool load_scene(std::string fname);
    IGL_INLINE bool save_scene();
    IGL_INLINE bool save_scene(std::string fname);
    // Draw everything
   // IGL_INLINE void draw();
    // OpenGL context resize
   
    // Helper functions

    IGL_INLINE void open_dialog_load_mesh();
    IGL_INLINE void open_dialog_save_mesh();

	IGL_INLINE void draw() {}
    ////////////////////////
    // Multi-mesh methods //
    ////////////////////////

    // Return the current mesh, or the mesh corresponding to a given unique identifier
    //
    // Inputs:
    //   mesh_id  unique identifier associated to the desired mesh (current mesh if -1)
    IGL_INLINE ViewerData& data(int mesh_id = -1);
    IGL_INLINE const ViewerData& data(int mesh_id = -1) const;

    // Append a new "slot" for a mesh (i.e., create empty entries at the end of
    // the data_list and opengl_state_list.
    //
    // Inputs:
    //   visible  If true, the new mesh is set to be visible on all existing viewports
    // Returns the id of the last appended mesh
    //
    // Side Effects:
    //   selected_data_index is set this newly created, last entry (i.e.,
    //   #meshes-1)
    IGL_INLINE int append_mesh(bool visible = true);

    // Erase a mesh (i.e., its corresponding data and state entires in data_list
    // and opengl_state_list)
    //
    // Inputs:
    //   index  index of mesh to erase
    // Returns whether erasure was successful <=> cannot erase last mesh
    //
    // Side Effects:
    //   If selected_data_index is greater than or equal to index then it is
    //   decremented
    // Example:
    //   // Erase all mesh slots except first and clear remaining mesh
    //   viewer.selected_data_index = viewer.data_list.size()-1;
    //   while(viewer.erase_mesh(viewer.selected_data_index)){};
    //   viewer.data().clear();
    //
    IGL_INLINE bool erase_mesh(const size_t index);

    // Retrieve mesh index from its unique identifier
    // Returns 0 if not found
    IGL_INLINE size_t mesh_index(const int id) const;


public:
    //////////////////////
    // Member variables //
    //////////////////////

    // Alec: I call this data_list instead of just data to avoid confusion with
    // old "data" variable.
    // Stores all the data that should be visualized
	bool run_ik = false;
	bool found_obj = false;	
	int arm_length = 4;
	double link_length = INT_MIN;
	Eigen::Vector4f arm_root;
	Eigen::Vector4f parent_axis_coordinates[26];
	Eigen::Matrix4f parent_axis_rotation[26];
	Eigen::Matrix4f arm_root_rotation;
	double arm_scale = 1.0f;
	Eigen::Vector3f arm_geo_center = Eigen::Vector3f::Zero();

    std::vector<ViewerData> data_list;
	struct ds {
		Eigen::MatrixXd V;
		Eigen::MatrixXi F;

		// Prepare array-based edge data structures and priority queue
		Eigen::VectorXi EMAP;
		Eigen::MatrixXi E, EF, EI;
		typedef std::set<std::pair<double, int> > PriorityQueue;
		PriorityQueue Q;
		std::vector<PriorityQueue::iterator > Qit;

		std::vector<Eigen::Matrix4d> Qv;

		// If an edge were collapsed, we'd collapse it to these points:
		Eigen::MatrixXd C;
		int num_collapsed;

		ds(igl::opengl::glfw::Viewer* viewer)
		{
			V = viewer->data().V;
			F = viewer->data().F;
			edge_flaps(F, E, EMAP, EF, EI);
			Qit.resize(E.rows());
			C.resize(E.rows(), V.cols());
			Eigen::VectorXd costs(E.rows());
			/*for (int e = 0;e < E.rows();e++)
			{
				double cost = e;
				Eigen::RowVectorXd p(1, 3);
				quadric_error_edge(e, V, F, E, EMAP, EF, EI, cost, p);
				C.row(e) = p;
				Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;
			}*/
			Qv = std::vector<Eigen::Matrix4d>(V.rows(), Eigen::Matrix4d::Zero());
			/*for (int v = 0; v < V.rows(); v++)
			{
				Qv.push_back(Eigen::Matrix4d::Zero());
			}*/
			num_collapsed = 0;
		}
	};

	std::vector<ds*> data_structures;


    size_t selected_data_index;
    int next_data_id;

    // List of registered plugins
//    std::vector<ViewerPlugin*> plugins;

    // Keep track of the global position of the scrollwheel
    float scroll_position;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // end namespace
} // end namespace
} // end namespace

#ifndef IGL_STATIC_LIBRARY
#  include "Viewer.cpp"
#endif

#endif
