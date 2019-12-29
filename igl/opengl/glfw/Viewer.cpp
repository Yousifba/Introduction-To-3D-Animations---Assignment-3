// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>
#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>
#include <igl/collapse_edge.h>

#include <igl/circulation.h>
#include <unordered_set>
#include <igl/edge_collapse_is_valid.h>
#include <my_collapse.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
namespace opengl
{
	namespace glfw
	{

		IGL_INLINE void Viewer::init()
		{


		}

		//IGL_INLINE void Viewer::init_plugins()
		//{
		//  // Init all plugins
		//  for (unsigned int i = 0; i<plugins.size(); ++i)
		//  {
		//    plugins[i]->init(this);
		//  }
		//}

		//IGL_INLINE void Viewer::shutdown_plugins()
		//{
		//  for (unsigned int i = 0; i<plugins.size(); ++i)
		//  {
		//    plugins[i]->shutdown();
		//  }
		//}

		IGL_INLINE Viewer::Viewer() :
			data_list(1),
			selected_data_index(0),
			next_data_id(1)
		{
			data_list.front().id = 0;



			// Temporary variables initialization
		   // down = false;
		  //  hack_never_moved = true;
			scroll_position = 0.0f;

			// Per face
			data().set_face_based(false);


#ifndef IGL_VIEWER_VIEWER_QUIET
			const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
			);
			std::cout << usage << std::endl;
#endif
		}

		IGL_INLINE Viewer::~Viewer()
		{
		}

		IGL_INLINE bool Viewer::load_mesh_from_file(
			const std::string& mesh_file_name_string)
		{

			// Create new data slot and set to selected
			if (!(data().F.rows() == 0 && data().V.rows() == 0))
			{
				append_mesh();
			}
			data().clear();

			size_t last_dot = mesh_file_name_string.rfind('.');
			if (last_dot == std::string::npos)
			{
				std::cerr << "Error: No file extension found in " <<
					mesh_file_name_string << std::endl;
				return false;
			}

			std::string extension = mesh_file_name_string.substr(last_dot + 1);

			if (extension == "off" || extension == "OFF")
			{
				Eigen::MatrixXd V;
				Eigen::MatrixXi F;
				if (!igl::readOFF(mesh_file_name_string, V, F))
					return false;
				data().set_mesh(V, F);
			}
			else if (extension == "obj" || extension == "OBJ")
			{
				Eigen::MatrixXd corner_normals;
				Eigen::MatrixXi fNormIndices;

				Eigen::MatrixXd UV_V;
				Eigen::MatrixXi UV_F;
				Eigen::MatrixXd V;
				Eigen::MatrixXi F;

				if (!(
					igl::readOBJ(
						mesh_file_name_string,
						V, UV_V, corner_normals, F, UV_F, fNormIndices)))
				{
					return false;
				}

				data().set_mesh(V, F);
				data().set_uv(UV_V, UV_F);

			}
			else
			{
				// unrecognized file type
				printf("Error: %s is not a recognized file type.\n", extension.c_str());
				return false;
			}

			data().compute_normals();
			data().uniform_colors(Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
				Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
				Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0));

			// Alec: why?
			if (data().V_uv.rows() == 0)
			{
				data().grid_texture();
			}


			//for (unsigned int i = 0; i<plugins.size(); ++i)
			//  if (plugins[i]->post_load())
			//    return true;

			return true;
		}

		IGL_INLINE bool Viewer::save_mesh_to_file(
			const std::string& mesh_file_name_string)
		{
			// first try to load it with a plugin
			//for (unsigned int i = 0; i<plugins.size(); ++i)
			//  if (plugins[i]->save(mesh_file_name_string))
			//    return true;

			size_t last_dot = mesh_file_name_string.rfind('.');
			if (last_dot == std::string::npos)
			{
				// No file type determined
				std::cerr << "Error: No file extension found in " <<
					mesh_file_name_string << std::endl;
				return false;
			}
			std::string extension = mesh_file_name_string.substr(last_dot + 1);
			if (extension == "off" || extension == "OFF")
			{
				return igl::writeOFF(
					mesh_file_name_string, data().V, data().F);
			}
			else if (extension == "obj" || extension == "OBJ")
			{
				Eigen::MatrixXd corner_normals;
				Eigen::MatrixXi fNormIndices;

				Eigen::MatrixXd UV_V;
				Eigen::MatrixXi UV_F;

				return igl::writeOBJ(mesh_file_name_string,
					data().V,
					data().F,
					corner_normals, fNormIndices, UV_V, UV_F);
			}
			else
			{
				// unrecognized file type
				printf("Error: %s is not a recognized file type.\n", extension.c_str());
				return false;
			}
			return true;
		}

		IGL_INLINE bool Viewer::load_scene()
		{
			std::string fname = igl::file_dialog_open();
			if (fname.length() == 0)
				return false;
			return load_scene(fname);
		}

		IGL_INLINE bool Viewer::load_scene(std::string fname)
		{
			// igl::deserialize(core(),"Core",fname.c_str());
			igl::deserialize(data(), "Data", fname.c_str());
			return true;
		}

		IGL_INLINE bool Viewer::save_scene()
		{
			std::string fname = igl::file_dialog_save();
			if (fname.length() == 0)
				return false;
			return save_scene(fname);
		}

		IGL_INLINE bool Viewer::save_scene(std::string fname)
		{
			//igl::serialize(core(),"Core",fname.c_str(),true);
			igl::serialize(data(), "Data", fname.c_str());

			return true;
		}

		IGL_INLINE void Viewer::open_dialog_load_mesh()
		{
			std::string fname = igl::file_dialog_open();

			if (fname.length() == 0)
				return;

			this->load_mesh_from_file(fname.c_str());
		}

		IGL_INLINE void Viewer::open_dialog_save_mesh()
		{
			std::string fname = igl::file_dialog_save();

			if (fname.length() == 0)
				return;

			this->save_mesh_to_file(fname.c_str());
		}

		IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
		{
			assert(!data_list.empty() && "data_list should never be empty");
			int index;
			if (mesh_id == -1)
				index = selected_data_index;
			else
				index = mesh_index(mesh_id);

			assert((index >= 0 && index < data_list.size()) &&
				"selected_data_index or mesh_id should be in bounds");
			return data_list[index];
		}

		IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
		{
			assert(!data_list.empty() && "data_list should never be empty");
			int index;
			if (mesh_id == -1)
				index = selected_data_index;
			else
				index = mesh_index(mesh_id);

			assert((index >= 0 && index < data_list.size()) &&
				"selected_data_index or mesh_id should be in bounds");
			return data_list[index];
		}

		IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
		{
			assert(data_list.size() >= 1);

			data_list.emplace_back();
			selected_data_index = data_list.size() - 1;
			data_list.back().id = next_data_id++;
			//if (visible)
			//    for (int i = 0; i < core_list.size(); i++)
			//        data_list.back().set_visible(true, core_list[i].id);
			//else
			//    data_list.back().is_visible = 0;
			return data_list.back().id;
		}

		IGL_INLINE bool Viewer::erase_mesh(const size_t index)
		{
			assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
			assert(data_list.size() >= 1);
			if (data_list.size() == 1)
			{
				// Cannot remove last mesh
				return false;
			}
			data_list[index].meshgl.free();
			data_list.erase(data_list.begin() + index);
			if (selected_data_index >= index && selected_data_index > 0)
			{
				selected_data_index--;
			}

			return true;
		}

		IGL_INLINE size_t Viewer::mesh_index(const int id) const {
			for (size_t i = 0; i < data_list.size(); ++i)
			{
				if (data_list[i].id == id)
					return i;
			}
			return 0;
		}

		int Viewer::sys_init()
		{
			load_meshs();
			int saved_index = selected_data_index;
			for (int i = 0; i < data_list.size(); i++)
			{
				selected_data_index = i;
				data().uniform_colors_index(1);
				load_meshs_ik();
			}
			selected_data_index = saved_index;
			return 0;	
		}

		int Viewer::load_meshs_ik()
		{
			Eigen::MatrixXd &V = data().V;
			Eigen::MatrixXi &F = data().F;
			// Load a mesh in OFF format
			//igl::readOFF(TUTORIAL_SHARED_PATH "/bunny.off", V, F);

			// Find the bounding box
			Eigen::Vector3d m = V.colwise().minCoeff();
			Eigen::Vector3d M = V.colwise().maxCoeff();



			if (selected_data_index == arm_length)
			{
				Eigen::MatrixXd tmp(1, 3);
				tmp <<
					(M(0) + m(0)) / 2, (M(2) + m(2)) / 2, (M(2) + m(2)) / 2;
				data().add_points(tmp.row(0), Eigen::RowVector3d(0, 0, 1));
				return 0;
			}

			link_length = abs(M(1) - m(1));

			// Corners of the bounding box
			Eigen::MatrixXd V_box(7, 3);
			V_box <<
				(M(0) + m(0)) / 2, M(1), (M(2) + m(2)) / 2,
				((M(0) + m(0)) / 2) + (link_length), M(1), (M(2) + m(2)) / 2, // x axis
				((M(0) + m(0)) / 2) - (link_length), M(1), (M(2) + m(2)) / 2,
				(M(0) + m(0)) / 2, M(1) + link_length, (M(2) + m(2)) / 2, // y axis
				(M(0) + m(0)) / 2, M(1), ((M(2) + m(2)) / 2) + link_length,	// z axis
				(M(0) + m(0)) / 2, M(1), ((M(2) + m(2)) / 2) - link_length,
				(M(0) + m(0)) / 2, m(1), ((M(2) + m(2)) / 2);

			// Edges of the bounding box
			Eigen::MatrixXi E_box(6, 2);
			E_box <<
				0, 1,
				0, 2,
				0, 3,
				0, 4,
				0, 5,
				0, 6;

			if (selected_data_index == 0)
			{
				arm_geo_center = Eigen::Vector3f((M(0) + m(0)) / 2, m(1), (M(2) + m(2)) / 2);
				parent_axis_coordinates[selected_data_index] = Eigen::Vector4f(0, 0.8, 0, 1);
				arm_root = Eigen::Vector4f(0, -0.8, 0, 1);
				arm_root_rotation = Eigen::Matrix4f::Identity();
			}
			else
			{
				data().MyTranslate(Eigen::Vector3f(0, link_length * (selected_data_index), 0), OBJECT_AXIS);
				parent_axis_coordinates[selected_data_index] = Eigen::Vector4f(0, 0.8 + link_length * (selected_data_index), 0, 1);
			}
			parent_axis_rotation[selected_data_index] = Eigen::Matrix4f::Identity();

			// Plot the mesh
			data().set_mesh(V, F);
			data().set_face_based(false);

			if (selected_data_index == arm_length - 1)
			{
				Eigen::MatrixXd tmp(1, 3);
				tmp <<
					(M(0) + m(0)) / 2, M(1), (M(2) + m(2)) / 2;
				data().add_points(tmp.row(0), Eigen::RowVector3d(0, 0, 1));
				return 0;
			}

			// Plot the corners of the bounding box as points
			data().add_points(V_box.row(0), Eigen::RowVector3d(0, 0, 1));


			Eigen::RowVector3d R = Eigen::RowVector3d(1, 0, 0), G = Eigen::RowVector3d(0, 1, 0), B = Eigen::RowVector3d(0, 0, 1);
			// Plot the edges of the bounding box
			for (unsigned i = 0;i < E_box.rows(); ++i)
				data().add_edges
				(
					V_box.row(E_box(i, 0)),
					V_box.row(E_box(i, 1)),
					(i < 2 ? R : (i == 2 || i == 5) ? G : B)
				);

			


			return 0;
		}

		int Viewer::load_meshs()
		{
			std::ifstream in("./configuration.txt");
			int cnt = 0;
			if (!in)
			{
				std::cout << "can't open file configuration.txt!" << std::endl;
				return 0;
			}

			char str[255];
			int line_index = 0;
			while (in)
			{
				in.getline(str, 255);
				if (in)
				{
					int n = 0;
					if (line_index == 0)
					{
						n = arm_length;
					}	
					else
					{
						n = 1;
					}
					for (int p = 0; p < n; p++)
					{
						if (!this->load_mesh_from_file(str))
						{
							//std::cout << ("Error loading mesh at %s .", str) << std::endl;
							return 0;
						}
						cnt++;
					}					
					line_index++;
				}
			}
			data().Move(Eigen::Vector4f(5, 0, 0, 1));
			return cnt;
		}

		IGL_INLINE bool Viewer::init_ds()
		{
			int saved_index = selected_data_index;

			for (int i = 0; i < data_list.size(); i++)
			{
				selected_data_index = i;
				ds* tmp = new ds(this);
				data_structures.push_back(tmp);
			}
			selected_data_index = saved_index;
			return true;
		}

		IGL_INLINE bool Viewer::collapse_5_percent(int n)
		{
			Eigen::MatrixXd& V = data_structures[selected_data_index]->V;
			Eigen::MatrixXi& F = data_structures[selected_data_index]->F;
			Eigen::MatrixXi& E = data_structures[selected_data_index]->E;
			Eigen::VectorXi& EMAP = data_structures[selected_data_index]->EMAP;
			Eigen::MatrixXi& EF = data_structures[selected_data_index]->EF;
			Eigen::MatrixXi& EI = data_structures[selected_data_index]->EI;
			typedef std::set<std::pair<double, int> > PriorityQueue;
			PriorityQueue& Q = data_structures[selected_data_index]->Q;
			std::vector<PriorityQueue::iterator >& Qit = data_structures[selected_data_index]->Qit;
			Eigen::MatrixXd& C = data_structures[selected_data_index]->C;
			int& num_collapsed = data_structures[selected_data_index]->num_collapsed;
			int edges_to_remove = std::ceil(0.05 * Q.size());
			bool something_collapsed = false;
			if (n == 0)
			{
				for (int i = 0; i < edges_to_remove; i++)
				{
					if (!my_collapse::my_collapse_e(V,
						F, E, EMAP, EF, EI, Q, Qit, C, this))
					{
						break;
					}
					something_collapsed = true;
					num_collapsed++;
				}
			}
			if (something_collapsed)
			{
				data().clear();
				data().set_mesh(V, F);
				data().set_face_based(true);
			}
			std::cout << "removed: " << num_collapsed << std::endl;
			return something_collapsed;
		}

		IGL_INLINE bool Viewer::collapse_edges(int num)
		{
			Eigen::MatrixXd& V = data_structures[selected_data_index]->V;
			Eigen::MatrixXi& F = data_structures[selected_data_index]->F;
			Eigen::VectorXi& EMAP = data_structures[selected_data_index]->EMAP;
			Eigen::MatrixXi& E = data_structures[selected_data_index]->E, & EF = data_structures[selected_data_index]->EF, & EI = data_structures[selected_data_index]->EI;
			typedef std::set<std::pair<double, int> > PriorityQueue;
			PriorityQueue& Q = data_structures[selected_data_index]->Q;
			std::vector<PriorityQueue::iterator >& Qit = data_structures[selected_data_index]->Qit;
			Eigen::MatrixXd& C = data_structures[selected_data_index]->C;
			int& num_collapsed = data_structures[selected_data_index]->num_collapsed;
			bool something_collapsed = false;

			for (int j = 0;j < num;j++)
			{
				const std::function<void(
					const int,
					const Eigen::MatrixXd&,
					const Eigen::MatrixXi&,
					const Eigen::MatrixXi&,
					const Eigen::VectorXi&,
					const Eigen::MatrixXi&,
					const Eigen::MatrixXi&,
					double&,
					Eigen::RowVectorXd&)>& cost_and_placement = shortest_edge_and_midpoint;

				if (!collapse_edge(
					shortest_edge_and_midpoint, V,
					F, E, EMAP, EF, EI, Q, Qit, C))
				{
					break;
				}
				something_collapsed = true;

				num_collapsed++;
			}
			return something_collapsed;
		}

		IGL_INLINE bool Viewer::quadric_error_handler()
		{
			int saved_index = selected_data_index;
			for (int i = 0; i < data_list.size(); i++)
			{
				selected_data_index = i;
				Eigen::MatrixXd* V = &data_structures[selected_data_index]->V;
				Eigen::MatrixXi* F = &data_structures[selected_data_index]->F;
				Eigen::VectorXi* EMAP = &data_structures[selected_data_index]->EMAP;
				Eigen::MatrixXi* E = &data_structures[selected_data_index]->E, * EF = &data_structures[selected_data_index]->EF, * EI = &data_structures[selected_data_index]->EI;
				typedef std::set<std::pair<double, int> > PriorityQueue;
				PriorityQueue* Q = &data_structures[selected_data_index]->Q;
				std::vector<PriorityQueue::iterator >* Qit = &data_structures[selected_data_index]->Qit;
				Eigen::MatrixXd* C = &data_structures[selected_data_index]->C;
				int* num_collapsed = &data_structures[selected_data_index]->num_collapsed;
				bool something_collapsed = false;
				double cost;
				Eigen::RowVectorXd p;
				quadric_error(1, *V, *F, *E, *EMAP, *EF, *EI, cost, p);
			}
			selected_data_index = saved_index;
			return true;
		}

		IGL_INLINE void Viewer::quadric_error(
			const int es,
			const Eigen::MatrixXd& V,
			const Eigen::MatrixXi& F,
			const Eigen::MatrixXi& E,
			const Eigen::VectorXi& EMAP,
			const Eigen::MatrixXi& EF,
			const Eigen::MatrixXi& EI,
			double& cost,
			Eigen::RowVectorXd& p)
		{
			typedef std::set<std::pair<double, int> > PriorityQueue;
			PriorityQueue& Q = data_structures[selected_data_index]->Q;
			std::vector<Eigen::Matrix4d>& Qv = data_structures[selected_data_index]->Qv;
			std::vector<PriorityQueue::iterator >& Qit = data_structures[selected_data_index]->Qit;
			Eigen::MatrixXd* C = &data_structures[selected_data_index]->C;
			std::vector<bool> check(V.rows(), false);
			for (int e = 0; e < E.rows(); e++)
			{
				std::vector<int> n_e0;
				std::vector<int> n_e1;
				if (!check.at(E(e, 0)))
				{
					n_e0 = igl::circulation(e, false, EMAP, EF, EI);
				}
				if (!check.at(E(e, 1)))
				{
					n_e1 = igl::circulation(e, true, EMAP, EF, EI);
				}			
				/*

				for (int f : n_e0_e1)
				{
					if (E(EMAP(f), 0) == E(e, 0) || E(EMAP(f), 1) == E(e, 0) ||
						E(EMAP(f + F.rows()), 0) == E(e, 0) || E(EMAP(f + F.rows()), 1) == E(e, 0) ||
						E(EMAP(f + 2 * F.rows()), 0) == E(e, 0) || E(EMAP(f + 2 * F.rows()), 1) == E(e, 0))
					{
						n_e0.push_back(f);
						std::cout << "0 " << f << std::endl;

					}
					if (E(EMAP(f), 0) == E(e, 1) || E(EMAP(f), 1) == E(e, 1) ||
						E(EMAP(f + F.rows()), 0) == E(e, 1) || E(EMAP(f + F.rows()), 1) == E(e, 1) ||
						E(EMAP(f + 2 * F.rows()), 0) == E(e, 1) || E(EMAP(f + 2 * F.rows()), 1) == E(e, 1))
					{
						n_e1.push_back(f);
						std::cout << "1 " << f << std::endl;

					}
				}*/

				Eigen::Matrix4d qv0 = Eigen::Matrix4d::Zero();
				Eigen::Matrix4d qv1 = Eigen::Matrix4d::Zero();

				Eigen::Vector3d current_vertex_pos = V.row(E(e, 0));
				double x = current_vertex_pos.x(), y = current_vertex_pos.y(), z = current_vertex_pos.z();
				if (!check.at(E(e, 0)))
				{
					for (int f : n_e0)
					{
						Eigen::Vector3d face_normal = data().F_normals.row(f).normalized();
						double a = face_normal.x(), b = face_normal.y(), c = face_normal.z();
						double d = -(a * x + b * y + c * z);
						Eigen::Vector4d plane = Eigen::Vector4d(a, b, c, d);
						Qv.at(E(e, 0)) += plane * plane.transpose();
					}
					check.at(E(e, 0)) = true;
					//Qv.at(E(e, 0)) = qv0;
				}
				qv0 = Qv.at(E(e, 0));

				current_vertex_pos = V.row(E(e, 1));
				x = current_vertex_pos.x(), y = current_vertex_pos.y(), z = current_vertex_pos.z();
				if (!check.at(E(e, 1)))
				{
					for (int f : n_e1)
					{
						Eigen::Vector3d face_normal = data().F_normals.row(f).normalized();
						double a = face_normal.x(), b = face_normal.y(), c = face_normal.z();
						double d = -(a * x + b * y + c * z);
						Eigen::Vector4d plane = Eigen::Vector4d(a, b, c, d);
						Qv.at(E(e, 1)) += plane * plane.transpose();
					}
					//Qv.at(E(e, 1)) = qv1;
					check.at(E(e, 1)) = true;
				}
				qv1 = Qv.at(E(e, 1));

							
				Eigen::Matrix4d q_t = qv0 + qv1;
				Eigen::Matrix4d q_t_inv;

				//q_t.row(3) = Eigen::Vector4d(0, 0, 0, 1);

				double tmp[] = { q_t(0, 0), q_t(0, 1), q_t(0, 2), q_t(0, 3),
								 q_t(0, 1), q_t(1, 1), q_t(1, 2), q_t(1, 3),
								 q_t(0, 2), q_t(1, 2), q_t(2, 2), q_t(2, 3),
								 0, 0, 0, 1 };

				Eigen::Matrix4d q_t2 = Eigen::Map<Eigen::Matrix4d>(tmp).transpose();

				bool inv;
				q_t2.computeInverseWithCheck(q_t_inv, inv);
				if (inv)
				{
					p = q_t_inv * Eigen::Vector4d(0, 0, 0, 1);
				}
				else
				{
					p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
				}
				p = Eigen::Vector3d(p.x(), p.y(), p.z());

				Eigen::Vector4d v_t = Eigen::Vector4d(p.x(), p.y(), p.z(), 1);
				cost = ((v_t.transpose() * q_t) * v_t);

				C->row(e) = p;
				Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;	
				/*if (selected_data_index == 1)
				{
					for (int f : n_e0)
						std::cout << f << " ";

					std::cout << std::endl;

					for (int f : n_e1)
						std::cout << f << " ";
					std::cout << std::endl;


					std::cout << cost << std::endl;
				}*/
			}
		}
		IGL_INLINE void Viewer::quadric_error_edge(
			const int e,
			const Eigen::MatrixXd& V,
			const Eigen::MatrixXi& F,
			const Eigen::MatrixXi& E,
			const Eigen::VectorXi& EMAP,
			const Eigen::MatrixXi& EF,
			const Eigen::MatrixXi& EI,
			double& cost,
			Eigen::RowVectorXd& p)
		{
			typedef std::set<std::pair<double, int> > PriorityQueue;
			PriorityQueue& Q = data_structures[selected_data_index]->Q;
			std::vector<Eigen::Matrix4d>& Qv = data_structures[selected_data_index]->Qv;
			std::vector<PriorityQueue::iterator >& Qit = data_structures[selected_data_index]->Qit;
			Eigen::MatrixXd* C = &data_structures[selected_data_index]->C;
			{
				std::vector<int> n_e0 = igl::circulation(e, false, EMAP, EF, EI);
				std::vector<int> n_e1 = igl::circulation(e, true, EMAP, EF, EI);
				/*
				for (int f : n_e0_e1)
				{
					if (E(EMAP(f), 0) == E(e, 0) || E(EMAP(f), 1) == E(e, 0) ||
						E(EMAP(f + F.rows()), 0) == E(e, 0) || E(EMAP(f + F.rows()), 1) == E(e, 0) ||
						E(EMAP(f + 2 * F.rows()), 0) == E(e, 0) || E(EMAP(f + 2 * F.rows()), 1) == E(e, 0))
					{
						n_e0.push_back(f);
						std::cout << "0 " << f << std::endl;

					}
					if (E(EMAP(f), 0) == E(e, 1) || E(EMAP(f), 1) == E(e, 1) ||
						E(EMAP(f + F.rows()), 0) == E(e, 1) || E(EMAP(f + F.rows()), 1) == E(e, 1) ||
						E(EMAP(f + 2 * F.rows()), 0) == E(e, 1) || E(EMAP(f + 2 * F.rows()), 1) == E(e, 1))
					{
						n_e1.push_back(f);
						std::cout << "1 " << f << std::endl;

					}
				}*/
				Eigen::Matrix4d qv0 = Eigen::Matrix4d::Zero();
				Eigen::Matrix4d qv1 = Eigen::Matrix4d::Zero();

				Eigen::Vector3d current_vertex_pos = V.row(E(e, 0));
				double x = current_vertex_pos.x(), y = current_vertex_pos.y(), z = current_vertex_pos.z();
				for (int f : n_e0)
				{
					Eigen::Vector3d face_normal = data().F_normals.row(f).normalized();
					double a = face_normal.x(), b = face_normal.y(), c = face_normal.z();
					double d = -(a * x + b * y + c * z);
					Eigen::Vector4d plane = Eigen::Vector4d(a, b, c, d);
					qv0 += plane * plane.transpose();
				}

				current_vertex_pos = V.row(E(e, 1));
				x = current_vertex_pos.x(), y = current_vertex_pos.y(), z = current_vertex_pos.z();
				for (int f : n_e1)
				{
					Eigen::Vector3d face_normal = data().F_normals.row(f).normalized();
					double a = face_normal.x(), b = face_normal.y(), c = face_normal.z();
					double d = -(a * x + b * y + c * z);
					Eigen::Vector4d plane = Eigen::Vector4d(a, b, c, d);
					qv1 += plane * plane.transpose();
				}		
				Eigen::Matrix4d q_t = qv0 + qv1;
				Eigen::Matrix4d q_t_inv;

				double tmp[] = { q_t(0, 0), q_t(0, 1), q_t(0, 2), q_t(0, 3),
								 q_t(0, 1), q_t(1, 1), q_t(1, 2), q_t(1, 3),
								 q_t(0, 2), q_t(1, 2), q_t(2, 2), q_t(2, 3),
								 0, 0, 0, 1 };

				Eigen::Matrix4d q_t2 = Eigen::Map<Eigen::Matrix4d>(tmp).transpose();

				bool inv;
				q_t2.computeInverseWithCheck(q_t_inv, inv);
				if (inv) // DID NOT WORK FOR ME!
				{
					p = q_t_inv * Eigen::Vector4d(0, 0, 0, 1); 
				}
				else
				{
					p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
				}	
				p = Eigen::Vector3d(p.x(), p.y(), p.z());

				Eigen::Vector4d v_t = Eigen::Vector4d(p.x(), p.y(), p.z(), 1);
				cost = ((v_t.transpose() * q_t) * v_t);		
			}
		}
	}
} // end namespace
} // end namespace
