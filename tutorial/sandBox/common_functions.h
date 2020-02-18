#pragma once

static void adjustModels(igl::opengl::glfw::Viewer* viewer) {
	int i;
	igl::opengl::ViewerData* curr = nullptr;

	for (i = 0; i < viewer->data_list.size(); i++) {
		curr = &viewer->data_list[i];
		if ((strcmp(&curr->model[0], "sphere"))) {		
			if (curr->son && curr->father) {
				curr->MyTranslate(Eigen::Vector3f(0, viewer->lenOfCy, 0));
			}
			// head of snake
			if (curr->head_of_snake) {
				curr->son->MyScale(Eigen::Vector3f(1, 0.5, 1));
				curr->son->MyTranslate(Eigen::Vector3f(0, -(viewer->lenOfCy * 0.5) / 2, 0));
				curr->MyTranslate(-Eigen::Vector3f(0, -(viewer->lenOfCy * 0.5) / 2, 0));
				curr->MyTranslate(Eigen::Vector3f(0, -viewer->lenOfCy / 2 + 0.3, 0));
				curr->MyScale(Eigen::Vector3f(1, 2, 1));
				curr->MyScale(Eigen::Vector3f(viewer->widOfCy / viewer->widOfSphere, 1.2, viewer->girthOfCy / viewer->girthOfSphere));
				viewer->lastCy = curr;
			}

			curr->SetCenterOfRotation(curr->bottom);

		}

	}
	// Adjusting the "camera"
	viewer->MyTranslate(Eigen::Vector3f(-5, -10, -30));
	//viewer->MyTranslate(Eigen::Vector3f(-1, -2, -7));
}