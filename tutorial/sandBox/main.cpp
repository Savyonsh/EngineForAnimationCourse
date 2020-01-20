
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "tutorial/sandBox/stb_image.h" // Texture
#include <iostream>
#include <fstream>
#include <string.h>
#include <vector>

using namespace std;

// Assignment 4 //
//bool read_Meshes(igl::opengl::glfw::Viewer* viewer, string file) {
//	ifstream conFile;
//	string line;
//	string model_name;
//	int times;
//	int selectedIndex = 0;
//	conFile.open(file, ios::in);
//	if (conFile.is_open()) {
//		while (getline(conFile, line)) {
//			viewer->load_mesh_from_file(line);			
//			viewer->load_mesh_from_file(line);
//			break;
//		}
//		conFile.close();		
//		return true;
//	}
//	else {
//		cout << "Error: No configuration. \n" << endl;
//		return false;
//	}
//}

//void adjustModels(igl::opengl::glfw::Viewer* viewer) {
//
//	for (auto i = 0; i < 2; i++) {
//		viewer->selected_data_index = i;
//		viewer->data().MyRotateX(2.5);
//		viewer->data().MyRotateY(0.35);
//		float val;
//		i == 0 ? val = 1.5 : val = -1.5;
//		viewer->data().Translate(Vector3f(val, 0, 0));
//		viewer->data().velocity = 0.05;
//		i == 0 ? val = -1 : val = 1;
//		viewer->data().direction = Vector3f(val, 0, 0);
//		viewer->data().show_lines = false;
//		i == 1 ? viewer->data().uniform_colors(Eigen::Vector3d(1.0, 0.0, 0.0),
//			Eigen::Vector3d(0.5, 0.3, 0.35),
//			Eigen::Vector3d(1.0, 0.5, 0.5)) : 0;
//		viewer->data().drawBox(viewer->data().tree.m_box, Eigen::RowVector3d(0, 255, 0));
//	}
//	// Adjusting the "camera"
//	viewer->MyTranslate(Vector3f(0, 0, -2));
//	// viewer->TranslateInSystem(viewer->MakeTrans(), Vector3f(0, 0, -2), true);
//}

// Assignment 3 //

bool read_Meshes(igl::opengl::glfw::Viewer* viewer, string file, int amountOfCy, int amountOfSh) {
	ifstream conFile;
	string line;
	string modelName;
	int times;
	int selectedIndex = 0;
	conFile.open(file, ios::in);
	if (conFile.is_open()) {
		while (getline(conFile, line)) {
			if (line.empty()) break;
			modelName = line;
			modelName.erase(modelName.find_last_of('.'));
			modelName.erase(0, modelName.find_last_of('\\') + 1);
			bool amISphere = !strcmp(&modelName[0], "sphere");
			!amISphere ? times = amountOfCy : times = amountOfSh;
			for (int i = 0; i < times; i++) {
				viewer->load_mesh_from_file(line);
				viewer->data_list[selectedIndex++].model = modelName;
			}
		}
		conFile.close();

		igl::opengl::ViewerData* curr = nullptr;
		igl::opengl::ViewerData* prev = nullptr;
		igl::opengl::ViewerData* headOfSnake = nullptr;
		int index_of_son = 0;

		for (int i = 0; i < viewer->data_list.size(); i++) {
			if (strcmp(&(viewer->data_list[i].model)[0], "sphere")) {
				curr = &viewer->data_list[i];
				curr->son = prev;
				if (prev) {
					prev->father = curr;
				}
				prev = curr;
				index_of_son = i;
			}
			else if(!headOfSnake) {
				headOfSnake = &viewer->data_list[i];
				headOfSnake->model = "ycylinder";
				headOfSnake->head_of_snake = true;

			}
		}
		// Connect sphere to last cylinder
		if (prev && headOfSnake) {
			headOfSnake->son = prev;
			prev->father = headOfSnake;
			headOfSnake->index_of_son = index_of_son;
		}

		return true;
	}
	else {
		cout << "Error: No configuration. \n" << endl;
		return false;
	}
}

void adjustModels(igl::opengl::glfw::Viewer* viewer, int times) {
	bool first = true;
	bool firstSphere = true;
	int i;
	float counter = 0;
	float counterSh = 0;
	float lenOfCy, girthOfCy, widOfCy;
	Eigen::Vector3d m;
	Eigen::Vector3d M;
	Eigen::MatrixXd axisPoints(6, 3);
	Eigen::MatrixXi axisLines(5, 2);
	Eigen::MatrixXd topBoxPoints(6, 3);
	Eigen::MatrixXi topBoxLines(12, 2);
	igl::opengl::ViewerData* curr = nullptr;

	for (i = 0; i < viewer->data_list.size(); i++) {
		curr = &viewer->data_list[i];
		curr->show_overlay_depth = false;

		if (!(strcmp(&curr->model[0], "sphere"))) {
			viewer->spheres.push_back(curr);
			curr->MyTranslate(Eigen::Vector3f(0, 5*counterSh++, 0));
			curr->direction = Eigen::Vector3f(0, -1, 0);
			curr->velocity = 0.1f;
			curr->show_lines = false;
			curr->bottomF = Vector4f(curr->V.colwise().minCoeff().cast<float>()(0),
				curr->V.colwise().minCoeff().cast<float>()(1),
				curr->V.colwise().minCoeff().cast<float>()(2), 1);
			m = curr->V.colwise().minCoeff();
			M = curr->V.colwise().maxCoeff();
			curr->top << 0, M(1), 0;
			curr->topF << 0, M(1), 0, 1;
		}
		else {
			counter++;
			m = curr->V.colwise().minCoeff();
			M = curr->V.colwise().maxCoeff();
			// Calculating only once, for the first cylinder.
			if (first) {
				axisPoints <<
					0, m(1), 0,							// Zero
					(M(0) + m(0)) / 2 + 1, m(1), 0,		// X
					-((M(0) + m(0)) / 2 + 1), m(1), 0,	// - X
					0, M(1) + 0.5, 0,					// Y
					0, m(1), (M(2) + m(2)) / 2 + 1,		// Z
					0, m(1), -((M(2) + m(2)) / 2 + 1);	// - Z

				axisLines <<
					0, 1,
					0, 2,
					0, 3,
					0, 4,
					0, 5;

				lenOfCy = M(1) - m(1);
				widOfCy = M(0) - m(0);
				girthOfCy = M(2) - m(2);
				viewer->lengthOfArm = lenOfCy * times;
				cout << viewer->lengthOfArm << endl;
				first = false;
			}

			curr->bottom << 0, m(1), 0;
			curr->top << 0, M(1), 0;
			curr->topF << 0, M(1), 0, 1;
			curr->bottomF << 0, m(1), 0, 1;
			curr->point_size = 2;
			curr->line_width = 0.5;
			curr->show_lines = false;
			//curr->add_points(axisPoints, Eigen::RowVector3d(1, 0, 0));

			//for (unsigned j = 0;j < axisLines.rows(); ++j) {
			//	curr->add_edges
			//	(
			//		axisPoints.row(axisLines(j, 0)),
			//		axisPoints.row(axisLines(j, 1)),
			//		Eigen::RowVector3d(0, 0, 1)
			//	);
			//}

			// in between
			if (curr->son && curr->father)
				curr->MyTranslate(Eigen::Vector3f(0, lenOfCy, 0));
			
			// head of snake
			if (!curr->father && curr->son) {
				curr->son->MyScale(Eigen::Vector3f(1, 0.5, 1));
				curr->son->MyTranslate(Eigen::Vector3f(0, -(lenOfCy * 0.5) / 2, 0));
				m = curr->V.colwise().minCoeff();
				M = curr->V.colwise().maxCoeff();
				float widOfSphere = M(0) - m(0);
				float girthOfSphere = M(2) - m(2);
				float lengthOfSphere = M(1) - m(0);
				curr->MyTranslate(-Eigen::Vector3f(0, -(lenOfCy * 0.5) / 2, 0));
				curr->MyTranslate(Eigen::Vector3f(0, -lenOfCy / 2 + 0.3, 0));
				curr->MyScale(Eigen::Vector3f(1, 2, 1));
				curr->MyScale(Eigen::Vector3f(widOfCy / widOfSphere, 1.2, girthOfCy / girthOfSphere));
			}

			curr->SetCenterOfRotation(curr->bottom);

		}

	}
	// Adjusting the "camera"
	//viewer->MyTranslate(Eigen::Vector3f(-8, -10, -30));
	//viewer->MyTranslate(Eigen::Vector3f(-1, -2, -7));
}

/*
* Explaing the function:
* image_path - will take a string, which show the path of the image file to load as a texture.
* data_ids = a list of ViewerData's ids (so we can use viewer.data_list[i]), each of those ids get the new texture.
* makeItBigger - if true, it will make the texture repeat itself less times, else, it will repeat itself more times.
* multi - a multiplier of how many times to make it repeat (doesn't have to an integer).
*/
void add_texture_to_list_of_datas(igl::opengl::glfw::Viewer& viewer, char* image_path, std::vector<int> data_ids, bool makeItBigger, double multi) {

	// Loading the image
	int width, height, n;
	unsigned char* data = stbi_load(image_path, &width, &height, &n, 4);
	if (data == nullptr) {
		std::cout << "Couldn't open the image at path: " << std::endl << image_path << std::endl;
		return;
	}
	Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(width, height), G(width, height), B(width, height), A(width, height);
	
	// Tamir version
	// It doesn't work if the image's height!=width
	/*for (unsigned int j = 0; j < height; ++j) {
		for (unsigned int i = 0; i < width; ++i) {
			R(j, i) = data[4 * ((width - 1 - i) + width * (height - 1 - j))];
			G(j, i) = data[4 * ((width - 1 - i) + width * (height - 1 - j)) + 1];
			B(j, i) = data[4 * ((width - 1 - i) + width * (height - 1 - j)) + 2];
			A(j, i) = data[4 * ((width - 1 - i) + width * (height - 1 - j)) + 3];
		}
	}*/
	
	// igl::png::readPNG version
	for (unsigned i = 0; i < height; ++i) {
		for (unsigned j = 0; j < width; ++j) {
			R(j, height - 1 - i) = data[4 * (j + width * i) + 0];
			G(j, height - 1 - i) = data[4 * (j + width * i) + 1];
			B(j, height - 1 - i) = data[4 * (j + width * i) + 2];
			A(j, height - 1 - i) = data[4 * (j + width * i) + 3];
		}
	}
	
	stbi_image_free(data); // data is stored in R/G/B/A, no need for data anymore.

	// Set the texture to each of the ViewerData in the list
	for (int& i : data_ids) {
		viewer.data_list[i].show_texture = true;

		if (makeItBigger)
			viewer.data_list[i].set_uv(viewer.data_list[i].V_uv / multi, viewer.data_list[i].F_uv / multi);
		else
			viewer.data_list[i].set_uv(viewer.data_list[i].V_uv * multi, viewer.data_list[i].F_uv * multi);

		viewer.data_list[i].set_texture(R, G, B, A);
		// viewer.data_list[i].grid_texture(); // What is it even for?
	}
}

int main(int argc, char* argv[])
{
	Display* disp = new Display(1000, 800, "Wellcome");
	Renderer renderer;
	igl::opengl::glfw::Viewer viewer;
	// Assignment 4 //

	//if (!(read_Meshes(&viewer, "configuration.txt"))) return 1;
	//adjustModels(&viewer);

	// Assignment 3 //

	int cyNum = 0, shNum = 3;
	// if (!(read_Meshes(&viewer, "configuration.txt", cyNum, shNum))) return 1;
	viewer.load_mesh_from_file("C:/Users/Sharon/source/repos/Project/tutorial/data/bunny.off");
	viewer.load_mesh_from_file("C:/Users/Sharon/source/repos/Project/tutorial/data/cube.obj");
	viewer.load_mesh_from_file("C:/Users/Sharon/source/repos/Project/tutorial/data/sphere.obj");
	/*viewer.load_mesh_from_file("E:/Users/Shaked/Documents/OneDrive/C++/GitHub/FinalProject/tutorial/data/bunny.off");
	viewer.load_mesh_from_file("E:/Users/Shaked/Documents/OneDrive/C++/GitHub/FinalProject/tutorial/data/cube.obj");
	viewer.load_mesh_from_file("E:/Users/Shaked/Documents/OneDrive/C++/GitHub/FinalProject/tutorial/data/sphere.obj");*/
	
	//adjustModels(&viewer, cyNum);
	//add_texture_to_list_of_datas(viewer, "SnakeSkin.png", std::vector<int>{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}, true, 5);
	
	Init(*disp);
	renderer.init(&viewer,800, 1000);
	
	disp->SetRenderer(&renderer);
	disp->launch_rendering(true);
	delete disp;
}