#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <Eigen/Dense>
Renderer::Renderer() : selected_core_index(0),
next_core_id(2)
{
	core_list.emplace_back(igl::opengl::ViewerCore());
	core_list.front().id = 1;
	// C-style callbacks
	callback_init = nullptr;
	callback_pre_draw = nullptr;
	callback_post_draw = nullptr;
	callback_mouse_down = nullptr;
	callback_mouse_up = nullptr;
	callback_mouse_move = nullptr;
	callback_mouse_scroll = nullptr;
	callback_key_down = nullptr;
	callback_key_up = nullptr;

	callback_init_data = nullptr;
	callback_pre_draw_data = nullptr;
	callback_post_draw_data = nullptr;
	callback_mouse_down_data = nullptr;
	callback_mouse_up_data = nullptr;
	callback_mouse_move_data = nullptr;
	callback_mouse_scroll_data = nullptr;
	callback_key_down_data = nullptr;
	callback_key_up_data = nullptr;
	highdpi = 1;

	xold = 0;
	yold = 0;

}

IGL_INLINE void Renderer::draw( GLFWwindow* window)
{
	using namespace std;
	using namespace Eigen;
	
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);
	
	auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

	if (fabs(highdpi_tmp - highdpi) > 1e-8)
	{
		post_resize(window,width, height);
		highdpi = highdpi_tmp;
	}

	for (auto& core : core_list)
	{
		core.clear_framebuffers();
	}

	for (auto& core : core_list)
	{
		for (auto& mesh : scn->data_list)
		{
			if (mesh.is_visible & core.id)
			{
				core.draw(scn->MakeTrans(),mesh);
			}
		}
	}
	if (scn->run_ik)
	{
		IK();
		//draw(window);
	}
}

void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
	core().init(); 

	core().align_camera_center(scn->data().V, scn->data().F);
}

void Renderer::IKUpdateTransformations(int root_index, float alpha, Eigen::Vector3f rot_vec)
{
	Eigen::Matrix4f parent_axis_translation = Eigen::Matrix4f();
	Eigen::Matrix4f parent_axis_rotation_prev = Eigen::Matrix4f();
	if (root_index > 0)
	{
		parent_axis_translation <<
			1, 0, 0, -scn->parent_axis_coordinates[root_index - 1].x(),
			0, 1, 0, -scn->parent_axis_coordinates[root_index - 1].y(),
			0, 0, 1, -scn->parent_axis_coordinates[root_index - 1].z(),
			0, 0, 0, 1;
		parent_axis_rotation_prev = scn->parent_axis_rotation[root_index - 1];
	}
	else
	{
		parent_axis_translation <<
			1, 0, 0, -scn->arm_root.x(),
			0, 1, 0, -scn->arm_root.y(),
			0, 0, 1, -scn->arm_root.z(),
			0, 0, 0, 1;
		parent_axis_rotation_prev = scn->arm_root_rotation;
	}
	Eigen::Matrix4f parent_axis_translation_inv = parent_axis_translation.inverse();
	Movable rot = Movable();
	rot.getTrans().rotate(Eigen::AngleAxisf(alpha, rot_vec));

	for (int i = root_index; i < scn->arm_length; i++) {
		scn->parent_axis_rotation[i] = rot.getTrans() * scn->parent_axis_rotation[i];
		scn->parent_axis_coordinates[i] = parent_axis_translation * scn->parent_axis_coordinates[i];
		scn->parent_axis_coordinates[i] = rot.getTrans().matrix() * scn->parent_axis_coordinates[i];
		scn->parent_axis_coordinates[i] = parent_axis_translation_inv * scn->parent_axis_coordinates[i];
	}
}

void Renderer::IKRotations(int root_index, float alpha, Eigen::Vector3f rot_vec)
{
	Eigen::Matrix4f parent_axis_translation = Eigen::Matrix4f();
	Eigen::Matrix4f parent_axis_rotation_prev = Eigen::Matrix4f();
	if (root_index > 0)
	{
		parent_axis_translation <<
			1, 0, 0, -scn->parent_axis_coordinates[root_index - 1].x(),
			0, 1, 0, -scn->parent_axis_coordinates[root_index - 1].y(),
			0, 0, 1, -scn->parent_axis_coordinates[root_index - 1].z(),
			0, 0, 0, 1;
		parent_axis_rotation_prev = scn->parent_axis_rotation[root_index - 1];
	}
	else
	{
		parent_axis_translation <<
			1, 0, 0, -scn->arm_root.x(),
			0, 1, 0, -scn->arm_root.y(),
			0, 0, 1, -scn->arm_root.z(),
			0, 0, 0, 1;
		parent_axis_rotation_prev = scn->arm_root_rotation;
	}
	Eigen::Matrix4f parent_axis_translation_inv = parent_axis_translation.inverse();

	Movable rot = Movable();
	rot.getTrans().rotate(Eigen::AngleAxisf(alpha, rot_vec));

	for (int i = root_index; i < scn->arm_length; i++) {

		scn->data_list[i].getTrans() = parent_axis_translation * scn->data_list[i].getTrans().matrix();

		scn->data_list[i].getTrans() = rot.getTrans().matrix() * scn->data_list[i].getTrans().matrix();
		scn->data_list[i].getTrans() = parent_axis_translation_inv * scn->data_list[i].getTrans().matrix();

	}
}

void Renderer::IK()
{
	Eigen::Vector3f dest = Eigen::Vector3f(scn->data_list[scn->arm_length].getTrans().matrix().col(3).x(),
		scn->data_list[scn->arm_length].getTrans().matrix().col(3).y(),
		scn->data_list[scn->arm_length].getTrans().matrix().col(3).z());
	Eigen::Vector3f root = Eigen::Vector3f(scn->arm_root.x(),
		scn->arm_root.y(),
		scn->arm_root.z());
	double distance_from_root = (double)( dest - root).norm();
	double arm_length = (double)scn->arm_length;
	double extended_arm = (arm_length) * scn->link_length * scn->arm_scale;
	double delta = 0.1f;
	if (distance_from_root - delta > extended_arm )
	{

		std::cout <<  "Can't Reach Destination! Short Of :\"" << distance_from_root - delta - extended_arm << "\"." << std::endl;
		scn->run_ik = false;
		return;
	}

	Eigen::Vector4f RE, RD;
	Eigen::Vector3f RE_3, RD_3;
	for (int i = scn->arm_length - 1; i >= 0; i--)
	{
		if (i > 0)
		{
			RE = (scn->parent_axis_coordinates[scn->arm_length - 1] - scn->parent_axis_coordinates[i - 1]);//.normalized();
			RD = (scn->data_list[scn->arm_length].getTrans().matrix().col(3) - scn->parent_axis_coordinates[i - 1]);//.normalized();
		}
		else
		{
			RE = (scn->parent_axis_coordinates[scn->arm_length - 1] - scn->arm_root);//.normalized();
			RD = (scn->data_list[scn->arm_length].getTrans().matrix().col(3) - scn->arm_root);//.normalized();
		}
		RE_3 = Eigen::Vector3f(RE.x(), RE.y(), RE.z());
		RD_3 = Eigen::Vector3f(RD.x(), RD.y(), RD.z());


		float cos_a = (RD_3.dot(RE_3)) / (RD_3.norm() * RE_3.norm());
		cos_a = fmin(1.0f, fmax(cos_a, -1.0f));
		float a = acos(cos_a);

		double distance = (scn->parent_axis_coordinates[scn->arm_length - 1] - scn->data_list[scn->arm_length].getTrans().matrix().col(3)).norm();		
		if (distance < delta)
		{
			scn->run_ik = false;
			std::cout << distance << std::endl;
			break;
		}

		Eigen::Vector3f tmp = Eigen::Vector3f(RE_3.cross(RD_3).x(), RE_3.cross(RD_3).y(), RE_3.cross(RD_3).z());
		IKRotations(i, a, tmp.normalized());
		IKUpdateTransformations(i, a, tmp.normalized());

		std::cout << distance << std::endl;
	}
}

void Renderer::UpdatePosition(double xpos, double ypos)
{
	xrel = xold - xpos;
	yrel = yold - ypos;
	xold = xpos;
	yold = ypos;
}

void Renderer::PrintArmsTips()
{
	float x, y, z;
	for (int i = 1; i <= scn->arm_length; i++)
	{
		x = scn->parent_axis_coordinates[i - 1].x(), y = scn->parent_axis_coordinates[i - 1].y(), z = scn->parent_axis_coordinates[i - 1].z();
		char* suffix = (i == 1) ? "st" : (i == 2) ? "nd" : (i == 3) ? "rd" : "th";
		printf("%d%s Arm Tip Position is: (%f, %f, %f).\n", i, suffix, x, y, z);
	}
}

void Renderer::PrintDestination()
{
	Eigen::Vector3f pos = scn->data_list[scn->arm_length].getTrans().translation();
	printf("Destination Position is: (%f, %f, %f).\n", pos.x(), pos.y(), pos.z());
}

void Renderer::TranslateCamera()
{
	GetScene()->MyTranslate(Eigen::Vector3f(-xrel / 250.0f, yrel / 250.0f, 0), CAMERA_AXIS);
}

void Renderer::RotateCamera()
{
	GetScene()->getTrans().prerotate(Eigen::AngleAxisf(-xrel / 180.0f, Eigen::Vector3f(0, 1, 0)));
	GetScene()->getTrans().prerotate(Eigen::AngleAxisf(-yrel / 180.0f, Eigen::Vector3f(1, 0, 0)));
	//GetScene()->MyRotate(Eigen::Vector3f(1, 0, 0), -yrel / 180.0f);
}

void Renderer::RotateArmLinkByKey(int key)
{
	switch (key)
	{
	case GLFW_KEY_LEFT:
		xrel = 5;
		yrel = 0;
		break;
	case GLFW_KEY_RIGHT:
		xrel = -5;
		yrel = 0;
		break;
	case GLFW_KEY_UP:
		xrel = 0;
		yrel = 5;
		break;
	case GLFW_KEY_DOWN:
		xrel = 0;
		yrel = -5;
		break;
	default:
		break;
	}
	if (scn->found_obj)
	{
		if (isArm())
		{
			RotateArmLinks(scn->selected_data_index);
			UpdateParentsTranslations(scn->selected_data_index);
		}	
		else
		{
			scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
			scn->data().MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
		}
	}
	else
	{
		RotateCamera();
	}
}

void Renderer::RotateArmLinks(int root_index)
{
	Eigen::Matrix4f parent_axis_translation = Eigen::Matrix4f();
	Eigen::Matrix4f parent_axis_rotation_prev = Eigen::Matrix4f();
	Eigen::Matrix4f parent_axis_rotation_cur = Eigen::Matrix4f();
	if (root_index > 0)
	{
		parent_axis_translation <<
			1, 0, 0, -scn->parent_axis_coordinates[root_index - 1].x(),
			0, 1, 0, -scn->parent_axis_coordinates[root_index - 1].y(),
			0, 0, 1, -scn->parent_axis_coordinates[root_index - 1].z(),
			0, 0, 0, 1;
		parent_axis_rotation_prev = scn->parent_axis_rotation[root_index - 1];
	}
	else
	{
		parent_axis_translation <<
			1, 0, 0, -scn->arm_root.x(),
			0, 1, 0, -scn->arm_root.y(),
			0, 0, 1, -scn->arm_root.z(),
			0, 0, 0, 1;
		parent_axis_rotation_prev = scn->arm_root_rotation;
	}
	Eigen::Matrix4f parent_axis_translation_inv = parent_axis_translation.inverse();

	parent_axis_rotation_cur = scn->parent_axis_rotation[root_index];

	for (int i = root_index; i < scn->arm_length; i++) {

		scn->data_list[i].getTrans() = parent_axis_translation * scn->data_list[i].getTrans().matrix();
		Eigen::Matrix4f x_rot = Eigen::Matrix4f();
		Eigen::Matrix4f y_rot = Eigen::Matrix4f();
		double theta_x = xrel / 180.0f;
		double theta_y = yrel / 180.0f;

		x_rot <<
			1, 0, 0, 0,
			0, cos(-theta_y), -sin(-theta_y), 0,
			0, sin(-theta_y), cos(-theta_y), 0,
			0, 0, 0, 1;

		y_rot <<
			cos(theta_x), 0, -sin(theta_x), 0,
			0, 1, 0, 0,
			sin(theta_x), 0, cos(theta_x), 0,
			0, 0, 0, 1;

		y_rot = parent_axis_rotation_prev * y_rot * parent_axis_rotation_prev.inverse();
		x_rot = parent_axis_rotation_cur * x_rot * parent_axis_rotation_cur.inverse();

		scn->data_list[i].getTrans() = parent_axis_translation_inv * x_rot * y_rot * scn->data_list[i].getTrans().matrix();
	}
}

void Renderer::UpdateParentsTranslations(int root_index)
{
	Eigen::Matrix4f parent_axis_translation = Eigen::Matrix4f();
	Eigen::Matrix4f parent_axis_rotation_prev = Eigen::Matrix4f();
	Eigen::Matrix4f parent_axis_rotation_cur = Eigen::Matrix4f();
	if (root_index > 0)
	{
		parent_axis_translation <<
			1, 0, 0, -scn->parent_axis_coordinates[root_index - 1].x(),
			0, 1, 0, -scn->parent_axis_coordinates[root_index - 1].y(),
			0, 0, 1, -scn->parent_axis_coordinates[root_index - 1].z(),
			0, 0, 0, 1;
		parent_axis_rotation_prev = scn->parent_axis_rotation[root_index - 1];
	}
	else
	{
		parent_axis_translation <<
			1, 0, 0, -scn->arm_root.x(),
			0, 1, 0, -scn->arm_root.y(),
			0, 0, 1, -scn->arm_root.z(),
			0, 0, 0, 1;
		parent_axis_rotation_prev = scn->arm_root_rotation;
	}
	Eigen::Matrix4f parent_axis_translation_inv = parent_axis_translation.inverse();
	Eigen::Matrix4f parent_axis_translation_cur = Eigen::Matrix4f();
	parent_axis_translation_cur <<
		1, 0, 0, -scn->parent_axis_coordinates[root_index].x(),
		0, 1, 0, -scn->parent_axis_coordinates[root_index].y(),
		0, 0, 1, -scn->parent_axis_coordinates[root_index].z(),
		0, 0, 0, 1;
	parent_axis_rotation_cur = scn->parent_axis_rotation[root_index];

	for (int i = root_index; i <=  scn->arm_length; i++) {
		scn->parent_axis_coordinates[i] = parent_axis_translation * scn->parent_axis_coordinates[i];
		Eigen::Matrix4f x_rot = Eigen::Matrix4f();
		Eigen::Matrix4f y_rot = Eigen::Matrix4f();
		double theta_x = xrel / 180.0f;
		double theta_y = yrel / 180.0f;

		x_rot <<
			1, 0, 0, 0,
			0, cos(-theta_y), -sin(-theta_y), 0,
			0, sin(-theta_y), cos(-theta_y), 0,
			0, 0, 0, 1;

		y_rot <<
			cos(theta_x), 0, -sin(theta_x), 0,
			0, 1, 0, 0,
			sin(theta_x), 0, cos(theta_x), 0,
			0, 0, 0, 1;

		x_rot = parent_axis_rotation_cur * x_rot * parent_axis_rotation_cur.inverse();
		y_rot = parent_axis_rotation_prev * y_rot * parent_axis_rotation_prev.inverse();
		scn->parent_axis_rotation[i] = x_rot * y_rot * scn->parent_axis_rotation[i];		

		scn->parent_axis_coordinates[i] = parent_axis_translation_inv * x_rot * y_rot * scn->parent_axis_coordinates[i];
	}
}

void Renderer::TranslateArm()
{
	Eigen::Vector3f translation_3(-xrel / 250.0f, yrel / 250.0f, 0);
	translation_3 = GetScene()->getTrans().rotation().inverse() * translation_3;
	Eigen::Vector4f translation_4(translation_3.x(), translation_3.y(), translation_3.z(), 0);
	for (int i = 0; i < scn->arm_length; i++)
	{		
		scn->data_list[i].MyTranslate(translation_3, CAMERA_AXIS);
		scn->parent_axis_coordinates[i] = scn->parent_axis_coordinates[i] + translation_4;
	}
	scn->arm_root = scn->arm_root + translation_4;
}

void Renderer::ScaleArm(double scale)
{	
	Eigen::Matrix4f parent_axis_translation = Eigen::Matrix4f();

	parent_axis_translation <<
		1, 0, 0, -scn->arm_root.x(),
		0, 1, 0, -scn->arm_root.y(),
		0, 0, 1, -scn->arm_root.z(),
		0, 0, 0, 1;

	Eigen::Matrix4f parent_axis_translation_inv = parent_axis_translation.inverse();

	for (int i = 0; i < scn->arm_length; i++) {

		scn->data_list[i].getTrans() = parent_axis_translation * scn->data_list[i].getTrans().matrix();
		Eigen::Matrix4f scale_mat = Eigen::Matrix4f();

		scale_mat <<
			scale, 0, 0, 0,
			0, scale, 0, 0,
			0, 0, scale, 0,
			0, 0, 0, 1;

		scn->data_list[i].getTrans() = parent_axis_translation_inv * scale_mat * scn->data_list[i].getTrans().matrix();
		scn->parent_axis_coordinates[i] = parent_axis_translation_inv * scale_mat * parent_axis_translation * scn->parent_axis_coordinates[i];
	}
	scn->arm_scale *= scale;
}

void Renderer::MouseProcessing(int button)
{
	if (button == GLFW_MOUSE_BUTTON_RIGHT)
	{
		if (isArm())
		{
			TranslateArm();
		}
		else
		{
			Eigen::Vector3f translation_3(-xrel / 250.0f, yrel / 250.0f, 0);
			translation_3 = GetScene()->getTrans().rotation().inverse() * translation_3;
			scn->data().MyTranslate(translation_3, CAMERA_AXIS);
		}		
	}
	else if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (isArm())
		{
			int root_index = scn->selected_data_index;
			RotateArmLinks(root_index);
			UpdateParentsTranslations(root_index);
		}
		else
		{
			scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
			scn->data().MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
	{
		scn->data().MyTranslate(Eigen::Vector3f(-xrel / 1000.0f, 0, 0), OBJECT_AXIS);
		scn->data().MyTranslate(Eigen::Vector3f(0, yrel / 1000.0f, 0), OBJECT_AXIS);
	}
}

/*Eigen::Vector4f Renderer::GetRootCoords()
{
	
}*/

bool Renderer::isArm()
{
	return
		scn->selected_data_index < scn->arm_length ? true : false;
}

Renderer::~Renderer()
{
	//if (scn)
	//	delete scn;
}

float Renderer::Picking(double newx, double newy)
{
		int fid;
		//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
		Eigen::Vector3f bc;
		double x = newx;
		double y = core().viewport(3) - newy;
		int p;
		Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
		igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
		view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
			* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTrans() * scn->data().MakeTrans();

		if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
			core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc))
		{
			Eigen::Vector3f click_pos = scn->data().V.row(scn->data().F.row(fid)[0]).cast<float>() * bc.x() + scn->data().V.row(scn->data().F.row(fid)[1]).cast<float>() * bc.y() + scn->data().V.row(scn->data().F.row(fid)[2]).cast<float>() * bc.z();;
			float tmp_array[] = { click_pos.x(), click_pos.y(), click_pos.z(), 1 };
			Eigen::Vector4f pos_vec = Eigen::Vector4f::Zero() + Eigen::Map<Eigen::Vector4f>(tmp_array);
			pos_vec = view * pos_vec;

			return  (pos_vec.z());
		}
		return INT_MIN;
}

IGL_INLINE void Renderer::resize(GLFWwindow* window,int w, int h)
	{
		if (window) {
			glfwSetWindowSize(window, w / highdpi, h / highdpi);
		}
		post_resize(window,w, h);
	}

	IGL_INLINE void Renderer::post_resize(GLFWwindow* window, int w, int h)
	{
		if (core_list.size() == 1)
		{
			core().viewport = Eigen::Vector4f(0, 0, w, h);
		}
		else
		{
			// It is up to the user to define the behavior of the post_resize() function
			// when there are multiple viewports (through the `callback_post_resize` callback)
		}
		//for (unsigned int i = 0; i < plugins.size(); ++i)
		//{
		//	plugins[i]->post_resize(w, h);
		//}
		if (callback_post_resize)
		{
			callback_post_resize(window, w, h);
		}
	}

	IGL_INLINE igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/)
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE const igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/) const
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE bool Renderer::erase_core(const size_t index)
	{
		assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
		//assert(data_list.size() >= 1);
		if (core_list.size() == 1)
		{
			// Cannot remove last viewport
			return false;
		}
		core_list[index].shut(); // does nothing
		core_list.erase(core_list.begin() + index);
		if (selected_core_index >= index && selected_core_index > 0)
		{
			selected_core_index--;
		}
		return true;
	}

	IGL_INLINE size_t Renderer::core_index(const int id) const {
		for (size_t i = 0; i < core_list.size(); ++i)
		{
			if (core_list[i].id == id)
				return i;
		}
		return 0;
	}

	IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
	{
		core_list.push_back(core()); // copies the previous active core and only changes the viewport
		core_list.back().viewport = viewport;
		core_list.back().id = next_core_id;
		next_core_id <<= 1;
		if (!append_empty)
		{
			for (auto& data : scn->data_list)
			{
				data.set_visible(true, core_list.back().id);
				//data.copy_options(core(), core_list.back());
			}
		}
		selected_core_index = core_list.size() - 1;
		return core_list.back().id;
	}

	//IGL_INLINE void Viewer::select_hovered_core()
	//{
	//	int width_window, height_window = 800;
	//   glfwGetFramebufferSize(window, &width_window, &height_window);
	//	for (int i = 0; i < core_list.size(); i++)
	//	{
	//		Eigen::Vector4f viewport = core_list[i].viewport;

	//		if ((current_mouse_x > viewport[0]) &&
	//			(current_mouse_x < viewport[0] + viewport[2]) &&
	//			((height_window - current_mouse_y) > viewport[1]) &&
	//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
	//		{
	//			selected_core_index = i;
	//			break;
	//		}
	//	}
	//}