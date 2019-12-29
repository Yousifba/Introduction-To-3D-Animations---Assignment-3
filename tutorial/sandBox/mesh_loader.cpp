#include "mesh_loader.h"
#include <iostream>
#include <fstream>


int load_mesh(igl::opengl::glfw::Viewer* viewer)
{
	std::ifstream in("./configuration.txt");

	if (!in)
	{
		std::cout << "can't open file configuration.txt!" << std::endl;
		return 1;
	}

	char str[255];

	while (in)
	{
		in.getline(str, 255);
		if (in)
		{
			viewer->load_mesh_from_file(str);
		}
	}

	in.close();

	return 0;
}