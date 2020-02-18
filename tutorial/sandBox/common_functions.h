#pragma once

static void adjustModels(igl::opengl::glfw::Viewer* viewer, int times, bool firstTime) {
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
			if (firstTime)
				viewer->spheres.push_back(curr);
			viewer->randomizeSphereLocation(curr);
			curr->velocityY = curr->velocityX = curr->velocityZ = 0.4;
			curr->move_model = true;
			curr->show_lines = false;
			//curr->should_appear = false;
			curr->bottomF = Vector4f(curr->V.colwise().minCoeff().cast<float>()(0),
									 curr->V.colwise().minCoeff().cast<float>()(1),
									 curr->V.colwise().minCoeff().cast<float>()(2), 1);
			m = curr->V.colwise().minCoeff();
			M = curr->V.colwise().maxCoeff();
			curr->top << 0, M(1), 0;
			curr->topF << 0, M(1), 0, 1;
		} else {
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
				viewer->firstCy = curr;
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
				viewer->lastCy = curr;
			}

			curr->SetCenterOfRotation(curr->bottom);

		}

	}
	// Adjusting the "camera"
	viewer->MyTranslate(Eigen::Vector3f(-5, -10, -30));
	//viewer->MyTranslate(Eigen::Vector3f(-1, -2, -7));
}