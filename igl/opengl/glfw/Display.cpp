#include <chrono>
#include <thread>
#include <ctime> // For the game - get current time

#include "../gl.h"
#include "Display.h"

#include "igl/igl_inline.h"
#include <igl/get_seconds.h>

static void glfw_error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

Display::Display(int windowWidth, int windowHeight, const std::string& title)
{
	bool resizable = true, fullscreen = false;
		glfwSetErrorCallback(glfw_error_callback);
		if (!glfwInit())
		{
			exit(EXIT_FAILURE);
		}
		glfwWindowHint(GLFW_SAMPLES, 8);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		
//#ifdef __APPLE__
//		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//#endif
//		if (fullscreen)
//		{
//			GLFWmonitor* monitor = glfwGetPrimaryMonitor();
//			const GLFWvidmode* mode = glfwGetVideoMode(monitor);
//			window = glfwCreateWindow(mode->width, mode->height, title.c_str(), monitor, nullptr);
//			windowWidth = mode->width;
//			windowHeight = mode->height;
//		}
//		else
//		{
			// Set default windows width
			//if (windowWidth <= 0 & core_list.size() == 1 && renderer->core().viewport[2] > 0)
			//	windowWidth = renderer->core().viewport[2];
			//else 
			//	if (windowWidth <= 0)
			//	windowWidth = 1280;
			//// Set default windows height
			//if (windowHeight <= 0 & core_list.size() == 1 && renderer->core().viewport[3] > 0)
			//	windowHeight = renderer->core().viewport[3];
			//else if (windowHeight <= 0)
			//	windowHeight = 800;
//			window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
//		}
		window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
		if (!window)
		{
			glfwTerminate();
			exit(EXIT_FAILURE);
		}
		glfwMakeContextCurrent(window);
		// Load OpenGL and its extensions
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			printf("Failed to load OpenGL and its extensions\n");
			exit(EXIT_FAILURE);
		}
#if defined(DEBUG) || defined(_DEBUG)
		printf("OpenGL Version %d.%d loaded\n", GLVersion.major, GLVersion.minor);
		int major, minor, rev;
		major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
		minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
		rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
		printf("OpenGL version received: %d.%d.%d\n", major, minor, rev);
		printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
		printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
#endif
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		//Tamir: changes from here
		// Initialize FormScreen
	   // __viewer = this;
		// Register callbacks
		//glfwSetKeyCallback(window, glfw_key_callback);
		//glfwSetCursorPosCallback(window,glfw_mouse_move);
		//glfwSetScrollCallback(window, glfw_mouse_scroll);
		//glfwSetMouseButtonCallback(window, glfw_mouse_press);
		//glfwSetWindowSizeCallback(window,glfw_window_size);
	
		
		//glfwSetCharModsCallback(window,glfw_char_mods_callback);
		//glfwSetDropCallback(window,glfw_drop_callback);
		// Handle retina displays (windows and mac)
		//int width, height;
		//glfwGetFramebufferSize(window, &width, &height);
		//int width_window, height_window;
		//glfwGetWindowSize(window, &width_window, &height_window);
		//highdpi = windowWidth/width_window;
		
		//glfw_window_size(window,width_window,height_window);
		//opengl.init();
//		core().align_camera_center(data().V, data().F);
		// Initialize IGL viewer
//		init();
		
}

static float my_distance(igl::opengl::glfw::Viewer* scn, igl::opengl::ViewerData* first) {
	Eigen::Vector3f destPoint = (scn->MakeTrans() * scn->data().MakeTrans() * Vector4f(0, 0, 0, 1)).block<3, 1>(0, 0);
	Eigen::Vector3f bottom = first->getBottomInWorld(scn->MakeTrans());
	return sqrt(pow(destPoint(0) - bottom(0), 2) +
				pow(destPoint(1) - bottom(1), 2) +
				pow(destPoint(2) - bottom(2), 2));
}

static void CalculateIK(igl::opengl::glfw::Viewer* scn, igl::opengl::ViewerData* last, Eigen::Vector3f destPoint) {
	// Start animation with the last cylinder
	// Calculating root and endpoint 
	igl::opengl::ViewerData* curr = nullptr;
	Eigen::Vector3f endpoint(last->getTopInWorld(scn->MakeTrans()));
	Eigen::Vector3f root(last->getBottomInWorld(scn->MakeTrans()));
	scn->Animate(root, endpoint, last, destPoint);

	curr = last->son;
	while (curr) {
		endpoint = last->getTopInWorld(scn->MakeTrans());
		root = curr->getBottomInWorld(scn->MakeTrans());
		scn->Animate(root, endpoint, curr, destPoint);
		curr = curr->son;
	}
}

bool Display::launch_rendering(bool loop)
{
	// glfwMakeContextCurrent(window);
	// Rendering loop
	const int num_extra_frames = 5;
	int frame_counter = 0;
	int windowWidth, windowHeight;
	//main loop
	Renderer* renderer = (Renderer*)glfwGetWindowUserPointer(window);
	glfwGetWindowSize(window, &windowWidth, &windowHeight);
	renderer->post_resize(window, windowWidth, windowHeight);
	igl::opengl::glfw::Viewer* scn = renderer->GetScene();

	// ------------- P R O J E C T ------------------ //
	igl::opengl::ViewerData* last = nullptr;
	igl::opengl::ViewerData* first = nullptr;
	igl::opengl::ViewerData* current = nullptr;
	igl::AABB<Eigen::MatrixXd, 3>* tree0, * tree1;
	igl::opengl::ViewerCore* core = &(((Renderer*)glfwGetWindowUserPointer(this->window))->core(2));
	Eigen::Matrix4d model0, model1;
	Eigen::Matrix3d Rot0, Rot1;
	vector<double> secondsPerSphere;
	Eigen::Vector3f destPoint, bottom;
	float distance;
	int index_top = 0;
	int i = 0;
	bool didIPrintAlready = false;

	// Finding the first and last cylinder
	for (unsigned int i = 0; i < scn->data_list.size(); i++) {
		if (strcmp(&scn->data_list[i].model[0], "sphere") && !scn->data_list[i].father) {
			last = scn->data_list[i].son;
			index_top = i;
		}
		if (strcmp(&scn->data_list[i].model[0], "sphere") && !scn->data_list[i].son) {
			first = &scn->data_list[i];
		}
	}	
	tree1 = &(scn->data_list[index_top].tree);

	// Starting the game	
	std::cout << "Round " << ++(renderer->round) << " is starting now!" << std::endl;
	renderer->round_start_time = std::chrono::system_clock::now(); 

	// Initializing time for spheres
	for (igl::opengl::ViewerData* sphere : scn->spheres) 
		secondsPerSphere.push_back(0);	
	
	while (!glfwWindowShouldClose(window))
	{
		// checking if round ended
		if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - renderer->round_start_time).count() < renderer->round_length) {
			// Falling animation for spheres 
			for (auto sphere : scn->spheres) {
				// sphere is out of range
				if (!sphere->should_appear)
				{
					scn->randomizeSphereLocation(sphere);
					sphere->velocityX = sphere->velocityY = sphere->velocityZ = 0.5;
					sphere->should_appear = true;
					sphere->move_model = true;
				}
				if (sphere->move_model) {
					// Move sphere
					sphere->Translate(Vector3f(sphere->direction(0) * sphere->velocityX,
						sphere->direction(1) * sphere->velocityY,
						sphere->direction(2) * sphere->velocityZ));

					// Check if sphere is in range (on screen) 
					Vector4f centerS = scn->MakeTrans() * sphere->MakeTrans() * Vector4f(0, 0, 0, 1);
					Vector3f centerF = first->getBottomInWorld(scn->MakeTrans());
					if (sqrt(pow(centerS(0) - centerF(0), 2) +
						pow(centerS(1) - centerF(1), 2) +
						pow(centerS(2) - centerF(2), 2)) >= 30) {
						sphere->should_appear = false;
						continue;
					}
					// Add falling effect if sphere moves in y direction
					if (abs(sphere->direction(1)) >= 0.4) {

						// Add gravity according to sphere's y direction
						(sphere->direction(1) < 0) ? sphere->velocityY += 0.07 : sphere->velocityY -= 0.1;
					}

					// Change direction of sphere if sphere isn't "zero" and touches the ground
					if (abs((scn->MakeTrans() * sphere->MakeTrans() * sphere->bottomF)(1)
						- first->getBottomInWorld(scn->MakeTrans())(1)) <= pow(1.0, -16)
						&& abs(sphere->velocityY) >= 0.3) {
						sphere->direction(1) = -sphere->direction(1);
					}

					if (abs((scn->MakeTrans() * sphere->MakeTrans() * sphere->bottomF)(1)
						- first->getBottomInWorld(scn->MakeTrans())(1)) <= pow(1.0, -16)
						&& abs(sphere->velocityY) < pow(1.0, -3)) {
						sphere->move_model = false;
						secondsPerSphere.at(i) = igl::get_seconds();
					}
				}

				// Checking how long sphere stayed on the ground
				else {
					if (secondsPerSphere.at(i) > 0 && abs(secondsPerSphere.at(i) - igl::get_seconds()) > 2)
						sphere->should_appear = false;
				}
				i++;
			}

			i = 0;
			if (scn->isIk) {
				// calculate the destenation point every loop as the selected shpere moves as well
				destPoint = (scn->MakeTrans() * scn->data().MakeTrans() * Vector4f(0, 0, 0, 1)).block<3, 1>(0, 0);
				bottom = first->getBottomInWorld(scn->MakeTrans());
				distance = sqrt(pow(destPoint(0) - bottom(0), 2) +
					pow(destPoint(1) - bottom(1), 2) +
					pow(destPoint(2) - bottom(2), 2));
				if (distance <= scn->lengthOfArm)
				{
					CalculateIK(scn, last, destPoint);
					first->UpdateCamera(core->camera_eye, core->camera_up, core->camera_translation, scn->MakeTrans());
					tree0 = &(scn->data().tree);
					tree1 = &(last->tree);
					model0 = scn->data().MakeTransD();
					model1 = first->MakeTransD();
					current = first;
					while (current->father) {
						current = current->father;
						model1 = model1 * current->MakeTransD();
					}
					Rot0 = model0.block<3, 3>(0, 0);
					Rot1 = model1.block<3, 3>(0, 0);
					if (scn->recursionIsIntersection(tree0, tree1, model0, model1, Rot0, Rot1)) {
						scn->isIk = false;
						scn->data().should_appear = false;
						scn->data().move_model = false;
						renderer->score += renderer->round * renderer->score_multi * 10;
						std::cout << "Your current score: " << renderer->score << std::endl;
					}
				}
				else {
					cout << "Distance too far." << endl;
					scn->isIk = false;
				}
			}
		} else {
			if (!didIPrintAlready) {
				std::cout << "Would you like to go another round? (Press Y/N)" << std::endl;
				didIPrintAlready = true;
				renderer->isGamePaused = true;
				scn->isIk = false;
				for (igl::opengl::ViewerData* sphere : scn->spheres) {
					sphere->move_model = false;
				}
			}
			
			if (renderer->flag_next_round == YES_NEXT_ROUND) {
				for (auto i = 0; i < scn->data_list.size(); i++) {
					// Reset the Position of all the objects
					scn->data_list[i].ResetMovable();

					if (!strcmp(&scn->data_list[i].model[0], "sphere")) {
						// Letting the balls move again
						scn->data_list[i].move_model = true;

						// TODO: Speed up the ball's velocity

					}
				}
				// set the snake & balls locations at the starting position
				scn->ResetMovable();
				adjustModels(scn, renderer->cyNum, false);
				first->UpdateCamera(core->camera_eye, core->camera_up, core->camera_translation, scn->MakeTrans());

				std::cout << "Round " << ++(renderer->round) << " is starting now!" << std::endl;

				// Setting up varibles for next round
				didIPrintAlready = false;
				renderer->flag_next_round = UNANSWERED_NEXT_ROUND;
				renderer->isGamePaused = false;
				renderer->round_start_time = std::chrono::system_clock::now();

			} else if (renderer->flag_next_round == NO_NEXT_ROUND) {
				std::cout << "Your final score: " << renderer->score << std::endl;
				return true; // Exiting the main loop, which will close the window as well.
			}
		}
	
		// ------------------------------------------------------------------------------- //
		
		double tic = igl::get_seconds();
			
		renderer->draw(window);
		glfwSwapBuffers(window);
		if (renderer->core().is_animating || frame_counter++ < num_extra_frames)
		{//motion
			glfwPollEvents();
			// In microseconds
			double duration = 1000000. * (igl::get_seconds() - tic);
			const double min_duration = 1000000. / renderer->core().animation_max_fps;
			if (duration < min_duration)
			{
				std::this_thread::sleep_for(std::chrono::microseconds((int)(min_duration - duration)));
			}
		}
		else
		{
			//glfwWaitEvents();
			glfwPollEvents();
			frame_counter = 0;
		}

		if (!loop)
			return !glfwWindowShouldClose(window);

#ifdef __APPLE__
		static bool first_time_hack = true;
		if (first_time_hack) {
			glfwHideWindow(window);
			glfwShowWindow(window);
			first_time_hack = false;
		}
#endif
	}
	return EXIT_SUCCESS;
}

void Display::AddKeyCallBack(void(*keyCallback)(GLFWwindow*, int, int, int, int))
{
	glfwSetKeyCallback(window, (void(*)(GLFWwindow*, int, int, int, int))keyCallback);//{

}

void Display::AddMouseCallBacks(void (*mousebuttonfun)(GLFWwindow*, int, int, int), void (*scrollfun)(GLFWwindow*, double, double), void (*cursorposfun)(GLFWwindow*, double, double))
{
	glfwSetMouseButtonCallback(window, mousebuttonfun);
	glfwSetScrollCallback(window, scrollfun);
	glfwSetCursorPosCallback(window, cursorposfun);
}

void Display::AddResizeCallBack(void (*windowsizefun)(GLFWwindow*, int, int))
{
	glfwSetWindowSizeCallback(window, windowsizefun);
}

void Display::SetRenderer(void* userPointer)
{
	
	glfwSetWindowUserPointer(window, userPointer);
	
}

void* Display::GetScene()
{
	return glfwGetWindowUserPointer(window);
}

void Display::SwapBuffers()
{
	glfwSwapBuffers(window);
}

void Display::PollEvents()
{
	glfwPollEvents();
}

Display::~Display()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}



