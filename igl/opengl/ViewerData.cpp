// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "ViewerData.h"
#include "ViewerCore.h"

#include "../per_face_normals.h"
#include "../material_colors.h"
#include "../parula.h"
#include "../per_vertex_normals.h"

#include <iostream>
#include <set>
#include <array>

#include <igl/collapse_edge.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/edge_flaps.h>


using namespace Eigen;
using namespace std;

IGL_INLINE igl::opengl::ViewerData::ViewerData()
: dirty(MeshGL::DIRTY_ALL),
  show_faces(true),
  show_lines(true),
  invert_normals(false),
  show_overlay(true),
  show_overlay_depth(true),
  show_vertid(false),
  show_faceid(false),
  show_texture(false),
  point_size(30),
  line_width(0.5f),
  line_color(0,0,0,1),
  label_color(0,0,0.04,1),
  shininess(35.0f),
  id(-1),
  is_visible(1)
{
  clear();
};

IGL_INLINE void igl::opengl::ViewerData::set_face_based(bool newvalue)
{
  if (face_based != newvalue)
  {
    face_based = newvalue;
    dirty = MeshGL::DIRTY_ALL;
  }
}

// Helpers that draws the most common meshes
IGL_INLINE void igl::opengl::ViewerData::set_mesh(
    const Eigen::MatrixXd& _V, const Eigen::MatrixXi& _F)
{
  using namespace std;

  Eigen::MatrixXd V_temp;

  // If V only has two columns, pad with a column of zeros
  if (_V.cols() == 2)
  {
    V_temp = Eigen::MatrixXd::Zero(_V.rows(),3);
    V_temp.block(0,0,_V.rows(),2) = _V;
  }
  else
    V_temp = _V;

  if (V.rows() == 0 && F.rows() == 0)
  {
    V = V_temp;
    F = _F;

    compute_normals();
    uniform_colors(
      Eigen::Vector3d(GOLD_AMBIENT[0], GOLD_AMBIENT[1], GOLD_AMBIENT[2]),
      Eigen::Vector3d(GOLD_DIFFUSE[0], GOLD_DIFFUSE[1], GOLD_DIFFUSE[2]),
      Eigen::Vector3d(GOLD_SPECULAR[0], GOLD_SPECULAR[1], GOLD_SPECULAR[2]));

    grid_texture();
  }
  else
  {
    if (_V.rows() == V.rows() && _F.rows() == F.rows())
    {
      V = V_temp;
      F = _F;
    }
    else
      cerr << "ERROR (set_mesh): The new mesh has a different number of vertices/faces. Please clear the mesh before plotting."<<endl;
  }
  dirty |= MeshGL::DIRTY_FACE | MeshGL::DIRTY_POSITION;
}

IGL_INLINE void igl::opengl::ViewerData::set_vertices(const Eigen::MatrixXd& _V)
{
  V = _V;
  assert(F.size() == 0 || F.maxCoeff() < V.rows());
  dirty |= MeshGL::DIRTY_POSITION;
}

IGL_INLINE void igl::opengl::ViewerData::set_normals(const Eigen::MatrixXd& N)
{
  using namespace std;
  if (N.rows() == V.rows())
  {
    set_face_based(false);
    V_normals = N;
  }
  else if (N.rows() == F.rows() || N.rows() == F.rows()*3)
  {
    set_face_based(true);
    F_normals = N;
  }
  else
    cerr << "ERROR (set_normals): Please provide a normal per face, per corner or per vertex."<<endl;
  dirty |= MeshGL::DIRTY_NORMAL;
}

IGL_INLINE void igl::opengl::ViewerData::set_visible(bool value, unsigned int core_id /*= 1*/)
{
  if (value)
    is_visible |= core_id;
  else
  is_visible &= ~core_id;
}

//IGL_INLINE void igl::opengl::ViewerData::copy_options(const ViewerCore &from, const ViewerCore &to)
//{
//  to.set(show_overlay      , from.is_set(show_overlay)      );
//  to.set(show_overlay_depth, from.is_set(show_overlay_depth));
//  to.set(show_texture      , from.is_set(show_texture)      );
//  to.set(show_faces        , from.is_set(show_faces)        );
//  to.set(show_lines        , from.is_set(show_lines)        );
//}


//---------------------------------------------------------------------------------------------

/*     A-S-S-I-G-N-M-E-N-T     2       */

IGL_INLINE void igl::opengl::ViewerData::decimate_by_size(int num_of_faces) {	
	MatrixXd V = this->V;
	MatrixXi F = this->F;
	int num_collapsed = 0;

	if (!Q.empty())
	{
		int j;									   
		for (j = 0;j < num_of_faces ;j++)
		{
			if (!collapse_edge(shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, Qit, C))	
				break;
			num_collapsed++;
		}

		if (num_collapsed > 0)
		{
			clear();
			set_mesh(V, F);
			set_face_based(true);
		}
	}
}

IGL_INLINE void igl::opengl::ViewerData::reset() {
	MatrixXd V = this->OV;
	MatrixXi F = this->OF;
	double cost; 

	edge_flaps(F, E, EMAP, EF, EI);	
	Qit.resize(E.rows());	
	C.resize(E.rows(), V.cols());	
	VectorXd costs(E.rows());
	Q.clear();

	//calculate_Kps(F, V);

	for (int e = 0;e < E.rows();e++)
	{
		//double cost = calculate_position_and_cost(e);
		//Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;

		cost = e;
		RowVectorXd p(1, 3);
		shortest_edge_and_midpoint(e, V, F, E, EMAP, EF, EI, cost, p);
		C.row(e) = p;
		Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;
	}

	clear();
	set_mesh(V, F);
	set_face_based(true);
}

IGL_INLINE void igl::opengl::ViewerData::calculate_Kps(MatrixXi& F, MatrixXd& V) {
	int f, v;
	vector<MatrixXd>* vertices_planes = new vector<MatrixXd>[V.rows()];

	for (f = 0; f < F.rows(); f++)
	{
		// Get normal of face
		Vector3d normal = F_normals.row(f).normalized();
		// Finding 'd' value in the plane equation
		double d = -normal(0) * V(F(f, 0), 0) - normal(1) * V(F(f, 0), 1) - normal(2) * V(F(f, 0), 2);
		Vector4d p(normal(0), normal(1), normal(2), d);
		// Calculating Kp with p * p^T
		MatrixXd Kp = p * p.transpose();
		// Inserting this Kp to each vertex list of K's in the triangle 
		vertices_planes[F(f, 0)].push_back(Kp);
		vertices_planes[F(f, 1)].push_back(Kp);
		vertices_planes[F(f, 2)].push_back(Kp);
	}

	for (int i = 0; i < V.rows(); i++)
	{
		MatrixXd Q = Matrix4d::Zero();
		vector<MatrixXd>::iterator it;
		// Sum all Kps
		for (it = vertices_planes[i].begin(); it != vertices_planes[i].end(); it++) {
			Q += *it;
		}
		this->vertices_planes.push_back(Q);

	}
	delete[] vertices_planes;
}

IGL_INLINE double igl::opengl::ViewerData::calculate_position_and_cost(int edge) {
	MatrixXd Q_tag = sum_Qs(edge);
	Eigen::FullPivLU<Matrix4d> Q_tag_check(Q_tag);
	Vector4d v_tag;

	if (Q_tag_check.isInvertible()) {
		Q_tag.row(3) << 0, 0, 0, 1;
		Vector4d temp(0, 0, 0, 1);
		temp = temp.transpose();
		v_tag = Q_tag.inverse() * temp;		
	}
	else {
		Vector3d vertex = V.row(E(edge, 0)) + V.row(E(edge, 1));
		vertex = vertex / 2;
		v_tag << vertex(0), vertex(1), vertex(2), 1;
	}
	// Update position of new vertex
	C.row(edge) << v_tag(0), v_tag(1), v_tag(2);

	// Return cost for combined edge
	return v_tag.transpose() * Q_tag * v_tag;
}

//IGL_INLINE bool igl::opengl::ViewerData::my_collapse_edge(MatrixXi& F, MatrixXd& V, bool& calQ) {
//#define v_tag E(cost_edge.second, 0)
//#define v_1 E(cost_edge.second, 0)
//#define v_2 E(cost_edge.second, 1)
//#define deleted_e cost_edge.second
//
//	if (Q.empty()) return false;
//
//	// Taking an edge out of the prioratyQ
//	pair<double, int> cost_edge = *Q.begin();
//	Q.erase(Q.begin());
//	Qit[deleted_e] = Q.end();
//
//	// Update F matrix 
//	//vector<int> N;
//	set<int> N;
//
//	int e, i, j;
//	for (e = 0; e < E.rows(); e++) { // Going trough all the edges
//		for (j = 0; j < 2; j++) { // Checking source and destination vertices
//			if (E(e, j) == v_2) {	
//				for (i = 0; i < 3; i++) { // Checking orianted and not in EF and updating F
//					if (F(EF(E(e, j), 0), i) == v_2) {
//						F(EF(E(e, j), 0), i) = v_tag;
//						N.insert(EF(E(e, j), 0));
//					}
//					if (F(EF(E(e, j), 1), i) == v_2) {
//						F(EF(E(e, j), 1), i) = v_tag;
//						N.insert(EF(E(e, j), 1));
//					}
//				}	
//				E(e, j) = v_tag;
//			}		
//		}
//	}
//
//	// Updating V matrix to hold the new vertex
//	Vector3d vertex = C.row(cost_edge.second);
//	V(v_tag, 0) = vertex(0);
//	V(v_tag, 1) = vertex(1);
//	V(v_tag, 2) = vertex(2);
//
//
//	// Update the priorityQ
//	for (auto n : N)
//	{
//		for (int v = 0;v < 3;v++)
//		{
//			// get edge id
//			const int ei = EMAP(v * F.rows() + n);
//			// erase old entry
//			Q.erase(Qit[ei]);
//			// compute cost and potential placement
//			double cost = calculate_position_and_cost(ei);
//			// Replace in queue
//			Qit[ei] = Q.insert(std::pair<double, int>(cost, ei)).first;
//		}
//
//	}
//
//	// Update vertices_planes with new combined Q
//	if (calQ) {
//		(*(vertices_planes.begin() + v_tag)) = sum_Qs(deleted_e);
//	}
//	// Updating data-bases.	
//	
//
//	//cout << "edge " << cost_edge.second <<
//	//	" , cost = " << cost_edge.first <<
//	//	" , new v position " <<
//	//	'( ' << vertex(0) << ' , ' << vertex(1) << ' , ' << vertex(2) << ' )' << endl;
//
//	return true;
//}

IGL_INLINE MatrixXd igl::opengl::ViewerData::sum_Qs(int edge) {
	int i;
	MatrixXd Q_tag; // The merged Q of the two Q's of the vertecies of the edge

	Q_tag = *(vertices_planes.begin() + E(edge, 0));
	Q_tag += *(vertices_planes.begin() + E(edge, 1));

	return Q_tag;
}

//---------------------------------------------------------------------------------------------
// Assignment 3

void igl::opengl::ViewerData::Translate(Eigen::Vector3f amt)
{
	if (!(strcmp(&model[0], "sphere")))
	{
		MyTranslate(amt);
		return;
	}
	if (son != nullptr) {
		igl::opengl::ViewerData* son = this->son;

		while (son->son != nullptr) {
			son = son->son;
		}
		son->MyTranslate(amt);
	}
	else {
		MyTranslate(amt);
	}
}

Eigen::Vector3f igl::opengl::ViewerData::getTopInWorld(Eigen::Matrix4f& world) {
	igl::opengl::ViewerData* son = this->son;
	igl::opengl::ViewerData* curr = nullptr;
	while (son) {
		curr = son;
		son = son->son;
	}
	Matrix4f trans(world);
	while (curr && curr != this) {
		trans = trans * curr->MakeTrans();
		curr = curr->father;
	}
	return (trans * MakeTrans() * topF).block<3, 1>(0, 0);
}

Eigen::Vector3f igl::opengl::ViewerData::getBottomInWorld(Eigen::Matrix4f& world) {
	igl::opengl::ViewerData* son = this->son;
	igl::opengl::ViewerData* curr = nullptr;
	while (son) {
		curr = son;
		son = son->son;
	}
	Matrix4f trans(world);
	while (curr && curr != this) {
		trans = trans * curr->MakeTrans();
		curr = curr->father;
	}
	return (trans * MakeTrans() * bottomF).block<3, 1>(0, 0);
}

// --------------------------------------------------------------------------------------------
// Assignment 4

void::igl::opengl::ViewerData::drawBox(AlignedBox<double, 3> m_box, RowVector3d color) {
	MatrixXd boxPoints(8, 3);
	Eigen::MatrixXi boxLines(12, 2);

	boxPoints.row(1) = m_box.corner(m_box.BottomRightFloor);
	boxPoints.row(2) = m_box.corner(m_box.TopRightFloor);
	boxPoints.row(5) = m_box.corner(m_box.BottomRightCeil);
	boxPoints.row(6) = m_box.corner(m_box.TopRightCeil);
	boxPoints.row(4) = m_box.corner(m_box.BottomLeftCeil);
	boxPoints.row(7) = m_box.corner(m_box.TopLeftCeil);
	boxPoints.row(0) = m_box.corner(m_box.BottomLeftFloor);
	boxPoints.row(3) = m_box.corner(m_box.TopLeftFloor);

	boxLines <<
		0, 1,
		1, 2,
		2, 3,
		3, 0,
		4, 5,
		5, 6,
		6, 7,
		7, 4,
		0, 4,
		1, 5,
		2, 6,
		7, 3;

	line_width = 2;
	point_size = 2;
	show_lines = false;

	// Plot the corners of the bounding box as points
	add_points(boxPoints, color);

	// Plot the edges of the bounding box
	for (unsigned i = 0;i < boxLines.rows(); ++i)
		add_edges
		(
			boxPoints.row(boxLines(i, 0)),
			boxPoints.row(boxLines(i, 1)),
			color
		);
}

void::igl::opengl::ViewerData::drawBoxes(igl::AABB<Eigen::MatrixXd, 3>* tree) {
	if (!tree) return;
	Vector3d top_edge(tree->m_box.corner(tree->m_box.TopRightCeil)
							  - tree->m_box.corner(tree->m_box.TopLeftCeil));
	double length_of_top_edge = sqrt(pow(top_edge(0),2) + pow(top_edge(1),2) + pow(top_edge(2),2));
	if (length_of_top_edge < 0.25)
		return;
	drawBox(tree->m_box, Eigen::RowVector3d::Random());
	//if (tree->m_left)
	//	drawBox(tree->m_left->m_box, Eigen::RowVector3d::Random());
	if (tree->m_right)
		drawBox(tree->m_right->m_box, Eigen::RowVector3d::Random().normalized());
	//drawBoxes(tree->m_left);
	drawBoxes(tree->m_right);
}

// --------------------------------------------------------------------------------------------

IGL_INLINE void igl::opengl::ViewerData::set_colors(const Eigen::MatrixXd &C)
{
  using namespace std;
  using namespace Eigen;
  if(C.rows()>0 && C.cols() == 1)
  {
    Eigen::MatrixXd C3;
    igl::parula(C,true,C3);
    return set_colors(C3);
  }
  // Ambient color should be darker color
  const auto ambient = [](const MatrixXd & C)->MatrixXd
  {
    MatrixXd T = 0.1*C;
    T.col(3) = C.col(3);
    return T;
  };
  // Specular color should be a less saturated and darker color: dampened
  // highlights
  const auto specular = [](const MatrixXd & C)->MatrixXd
  {
    const double grey = 0.3;
    MatrixXd T = grey+0.1*(C.array()-grey);
    T.col(3) = C.col(3);
    return T;
  };
  if (C.rows() == 1)
  {
    for (unsigned i=0;i<V_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        V_material_diffuse.row(i) << C.row(0),1;
      else if (C.cols() == 4)
        V_material_diffuse.row(i) << C.row(0);
    }
    V_material_ambient = ambient(V_material_diffuse);
    V_material_specular = specular(V_material_diffuse);

    for (unsigned i=0;i<F_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        F_material_diffuse.row(i) << C.row(0),1;
      else if (C.cols() == 4)
        F_material_diffuse.row(i) << C.row(0);
    }
    F_material_ambient = ambient(F_material_diffuse);
    F_material_specular = specular(F_material_diffuse);
  }
  else if (C.rows() == V.rows())
  {
    set_face_based(false);
    for (unsigned i=0;i<V_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        V_material_diffuse.row(i) << C.row(i), 1;
      else if (C.cols() == 4)
        V_material_diffuse.row(i) << C.row(i);
    }
    V_material_ambient = ambient(V_material_diffuse);
    V_material_specular = specular(V_material_diffuse);
  }
  else if (C.rows() == F.rows())
  {
    set_face_based(true);
    for (unsigned i=0;i<F_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        F_material_diffuse.row(i) << C.row(i), 1;
      else if (C.cols() == 4)
        F_material_diffuse.row(i) << C.row(i);
    }
    F_material_ambient = ambient(F_material_diffuse);
    F_material_specular = specular(F_material_diffuse);
  }
  else
    cerr << "ERROR (set_colors): Please provide a single color, or a color per face or per vertex."<<endl;
  dirty |= MeshGL::DIRTY_DIFFUSE;

}

IGL_INLINE void igl::opengl::ViewerData::set_uv(const Eigen::MatrixXd& UV)
{
  using namespace std;
  if (UV.rows() == V.rows())
  {
    set_face_based(false);
    V_uv = UV;
  }
  else
    cerr << "ERROR (set_UV): Please provide uv per vertex."<<endl;;
  dirty |= MeshGL::DIRTY_UV;
}

IGL_INLINE void igl::opengl::ViewerData::set_uv(const Eigen::MatrixXd& UV_V, const Eigen::MatrixXi& UV_F)
{
  set_face_based(true);
  V_uv = UV_V.block(0,0,UV_V.rows(),2);
  F_uv = UV_F;
  dirty |= MeshGL::DIRTY_UV;
}

IGL_INLINE void igl::opengl::ViewerData::set_texture(
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B)
{
  texture_R = R;
  texture_G = G;
  texture_B = B;
  texture_A = Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>::Constant(R.rows(),R.cols(),255);
  dirty |= MeshGL::DIRTY_TEXTURE;
}

IGL_INLINE void igl::opengl::ViewerData::set_texture(
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& A)
{
  texture_R = R;
  texture_G = G;
  texture_B = B;
  texture_A = A;
  dirty |= MeshGL::DIRTY_TEXTURE;
}

IGL_INLINE void igl::opengl::ViewerData::set_points(
  const Eigen::MatrixXd& P,
  const Eigen::MatrixXd& C)
{
  // clear existing points
  points.resize(0,0);
  add_points(P,C);
}

IGL_INLINE void igl::opengl::ViewerData::add_points(const Eigen::MatrixXd& P,  const Eigen::MatrixXd& C)
{
  Eigen::MatrixXd P_temp;

  // If P only has two columns, pad with a column of zeros
  if (P.cols() == 2)
  {
    P_temp = Eigen::MatrixXd::Zero(P.rows(),3);
    P_temp.block(0,0,P.rows(),2) = P;
  }
  else
    P_temp = P;

  int lastid = points.rows();
  points.conservativeResize(points.rows() + P_temp.rows(),6);
  for (unsigned i=0; i<P_temp.rows(); ++i)
    points.row(lastid+i) << P_temp.row(i), i<C.rows() ? C.row(i) : C.row(C.rows()-1);

  dirty |= MeshGL::DIRTY_OVERLAY_POINTS;
}

IGL_INLINE void igl::opengl::ViewerData::set_edges(
  const Eigen::MatrixXd& P,
  const Eigen::MatrixXi& E,
  const Eigen::MatrixXd& C)
{
  using namespace Eigen;
  lines.resize(E.rows(),9);
  assert(C.cols() == 3);
  for(int e = 0;e<E.rows();e++)
  {
    RowVector3d color;
    if(C.size() == 3)
    {
      color<<C;
    }else if(C.rows() == E.rows())
    {
      color<<C.row(e);
    }
    lines.row(e)<< P.row(E(e,0)), P.row(E(e,1)), color;
  }
  dirty |= MeshGL::DIRTY_OVERLAY_LINES;
}

IGL_INLINE void igl::opengl::ViewerData::add_edges(const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2, const Eigen::MatrixXd& C)
{
  Eigen::MatrixXd P1_temp,P2_temp;

  // If P1 only has two columns, pad with a column of zeros
  if (P1.cols() == 2)
  {
    P1_temp = Eigen::MatrixXd::Zero(P1.rows(),3);
    P1_temp.block(0,0,P1.rows(),2) = P1;
    P2_temp = Eigen::MatrixXd::Zero(P2.rows(),3);
    P2_temp.block(0,0,P2.rows(),2) = P2;
  }
  else
  {
    P1_temp = P1;
    P2_temp = P2;
  }

  int lastid = lines.rows();
  lines.conservativeResize(lines.rows() + P1_temp.rows(),9);
  for (unsigned i=0; i<P1_temp.rows(); ++i)
    lines.row(lastid+i) << P1_temp.row(i), P2_temp.row(i), i<C.rows() ? C.row(i) : C.row(C.rows()-1);

  dirty |= MeshGL::DIRTY_OVERLAY_LINES;
}

IGL_INLINE void igl::opengl::ViewerData::add_label(const Eigen::VectorXd& P,  const std::string& str)
{
  Eigen::RowVectorXd P_temp;

  // If P only has two columns, pad with a column of zeros
  if (P.size() == 2)
  {
    P_temp = Eigen::RowVectorXd::Zero(3);
    P_temp << P.transpose(), 0;
  }
  else
    P_temp = P;

  int lastid = labels_positions.rows();
  labels_positions.conservativeResize(lastid+1, 3);
  labels_positions.row(lastid) = P_temp;
  labels_strings.push_back(str);
}

IGL_INLINE void igl::opengl::ViewerData::clear_labels()
{
  labels_positions.resize(0,3);
  labels_strings.clear();
}

IGL_INLINE void igl::opengl::ViewerData::clear()
{
  V                       = Eigen::MatrixXd (0,3);
  F                       = Eigen::MatrixXi (0,3);

  F_material_ambient      = Eigen::MatrixXd (0,4);
  F_material_diffuse      = Eigen::MatrixXd (0,4);
  F_material_specular     = Eigen::MatrixXd (0,4);

  V_material_ambient      = Eigen::MatrixXd (0,4);
  V_material_diffuse      = Eigen::MatrixXd (0,4);
  V_material_specular     = Eigen::MatrixXd (0,4);

  F_normals               = Eigen::MatrixXd (0,3);
  V_normals               = Eigen::MatrixXd (0,3);

  V_uv                    = Eigen::MatrixXd (0,2);
  F_uv                    = Eigen::MatrixXi (0,3);

  lines                   = Eigen::MatrixXd (0,9);
  points                  = Eigen::MatrixXd (0,6);
  labels_positions        = Eigen::MatrixXd (0,3);
  labels_strings.clear();

  face_based = false;
}

IGL_INLINE void igl::opengl::ViewerData::compute_normals()
{
  igl::per_face_normals(V, F, F_normals);
  igl::per_vertex_normals(V, F, F_normals, V_normals);
  dirty |= MeshGL::DIRTY_NORMAL;
}

IGL_INLINE void igl::opengl::ViewerData::uniform_colors(
  const Eigen::Vector3d& ambient,
  const Eigen::Vector3d& diffuse,
  const Eigen::Vector3d& specular)
{
  Eigen::Vector4d ambient4;
  Eigen::Vector4d diffuse4;
  Eigen::Vector4d specular4;

  ambient4 << ambient, 1;
  diffuse4 << diffuse, 1;
  specular4 << specular, 1;

  uniform_colors(ambient4,diffuse4,specular4);
}

IGL_INLINE void igl::opengl::ViewerData::uniform_colors(
  const Eigen::Vector4d& ambient,
  const Eigen::Vector4d& diffuse,
  const Eigen::Vector4d& specular)
{
  V_material_ambient.resize(V.rows(),4);
  V_material_diffuse.resize(V.rows(),4);
  V_material_specular.resize(V.rows(),4);

  for (unsigned i=0; i<V.rows();++i)
  {
    V_material_ambient.row(i) = ambient;
    V_material_diffuse.row(i) = diffuse;
    V_material_specular.row(i) = specular;
  }

  F_material_ambient.resize(F.rows(),4);
  F_material_diffuse.resize(F.rows(),4);
  F_material_specular.resize(F.rows(),4);

  for (unsigned i=0; i<F.rows();++i)
  {
    F_material_ambient.row(i) = ambient;
    F_material_diffuse.row(i) = diffuse;
    F_material_specular.row(i) = specular;
  }
  dirty |= MeshGL::DIRTY_SPECULAR | MeshGL::DIRTY_DIFFUSE | MeshGL::DIRTY_AMBIENT;
}

IGL_INLINE void igl::opengl::ViewerData::grid_texture()
{
  // Don't do anything for an empty mesh
  if(V.rows() == 0)
  {
    V_uv.resize(V.rows(),2);
    return;
  }
  if (V_uv.rows() == 0)
  {
    V_uv = V.block(0, 0, V.rows(), 2);
    V_uv.col(0) = V_uv.col(0).array() - V_uv.col(0).minCoeff();
    V_uv.col(0) = V_uv.col(0).array() / V_uv.col(0).maxCoeff();
    V_uv.col(1) = V_uv.col(1).array() - V_uv.col(1).minCoeff();
    V_uv.col(1) = V_uv.col(1).array() / V_uv.col(1).maxCoeff();
    V_uv = V_uv.array() * 10;
    dirty |= MeshGL::DIRTY_TEXTURE;
  }

  unsigned size = 128;
  unsigned size2 = size/2;
  texture_R.resize(size, size);
  for (unsigned i=0; i<size; ++i)
  {
    for (unsigned j=0; j<size; ++j)
    {
      texture_R(i,j) = 0;
      if ((i<size2 && j<size2) || (i>=size2 && j>=size2))
        texture_R(i,j) = 255;
    }
  }

  texture_G = texture_R;
  texture_B = texture_R;
  texture_A = Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>::Constant(texture_R.rows(),texture_R.cols(),255);
  dirty |= MeshGL::DIRTY_TEXTURE;
}

IGL_INLINE void igl::opengl::ViewerData::updateGL(
  const igl::opengl::ViewerData& data,
  const bool invert_normals,
  igl::opengl::MeshGL& meshgl
  )
{
  if (!meshgl.is_initialized)
  {
    meshgl.init();
  }

  bool per_corner_uv = (data.F_uv.rows() == data.F.rows());
  bool per_corner_normals = (data.F_normals.rows() == 3 * data.F.rows());

  meshgl.dirty |= data.dirty;

  // Input:
  //   X  #F by dim quantity
  // Output:
  //   X_vbo  #F*3 by dim scattering per corner
  const auto per_face = [&data](
      const Eigen::MatrixXd & X,
      MeshGL::RowMatrixXf & X_vbo)
  {
    assert(X.cols() == 4);
    X_vbo.resize(data.F.rows()*3,4);
    for (unsigned i=0; i<data.F.rows();++i)
      for (unsigned j=0;j<3;++j)
        X_vbo.row(i*3+j) = X.row(i).cast<float>();
  };

  // Input:
  //   X  #V by dim quantity
  // Output:
  //   X_vbo  #F*3 by dim scattering per corner
  const auto per_corner = [&data](
      const Eigen::MatrixXd & X,
      MeshGL::RowMatrixXf & X_vbo)
  {
    X_vbo.resize(data.F.rows()*3,X.cols());
    for (unsigned i=0; i<data.F.rows();++i)
      for (unsigned j=0;j<3;++j)
        X_vbo.row(i*3+j) = X.row(data.F(i,j)).cast<float>();
  };

  if (!data.face_based)
  {
    if (!(per_corner_uv || per_corner_normals))
    {
      // Vertex positions
      if (meshgl.dirty & MeshGL::DIRTY_POSITION)
        meshgl.V_vbo = data.V.cast<float>();

      // Vertex normals
      if (meshgl.dirty & MeshGL::DIRTY_NORMAL)
      {
        meshgl.V_normals_vbo = data.V_normals.cast<float>();
        if (invert_normals)
          meshgl.V_normals_vbo = -meshgl.V_normals_vbo;
      }

      // Per-vertex material settings
      if (meshgl.dirty & MeshGL::DIRTY_AMBIENT)
        meshgl.V_ambient_vbo = data.V_material_ambient.cast<float>();
      if (meshgl.dirty & MeshGL::DIRTY_DIFFUSE)
        meshgl.V_diffuse_vbo = data.V_material_diffuse.cast<float>();
      if (meshgl.dirty & MeshGL::DIRTY_SPECULAR)
        meshgl.V_specular_vbo = data.V_material_specular.cast<float>();

      // Face indices
      if (meshgl.dirty & MeshGL::DIRTY_FACE)
        meshgl.F_vbo = data.F.cast<unsigned>();

      // Texture coordinates
      if (meshgl.dirty & MeshGL::DIRTY_UV)
      {
        meshgl.V_uv_vbo = data.V_uv.cast<float>();
      }
    }
    else
    {

      // Per vertex properties with per corner UVs
      if (meshgl.dirty & MeshGL::DIRTY_POSITION)
      {
        per_corner(data.V,meshgl.V_vbo);
      }

      if (meshgl.dirty & MeshGL::DIRTY_AMBIENT)
      {
        meshgl.V_ambient_vbo.resize(data.F.rows()*3,4);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_ambient_vbo.row(i*3+j) = data.V_material_ambient.row(data.F(i,j)).cast<float>();
      }
      if (meshgl.dirty & MeshGL::DIRTY_DIFFUSE)
      {
        meshgl.V_diffuse_vbo.resize(data.F.rows()*3,4);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_diffuse_vbo.row(i*3+j) = data.V_material_diffuse.row(data.F(i,j)).cast<float>();
      }
      if (meshgl.dirty & MeshGL::DIRTY_SPECULAR)
      {
        meshgl.V_specular_vbo.resize(data.F.rows()*3,4);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_specular_vbo.row(i*3+j) = data.V_material_specular.row(data.F(i,j)).cast<float>();
      }

      if (meshgl.dirty & MeshGL::DIRTY_NORMAL)
      {
        meshgl.V_normals_vbo.resize(data.F.rows()*3,3);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_normals_vbo.row(i*3+j) =
                         per_corner_normals ?
               data.F_normals.row(i*3+j).cast<float>() :
               data.V_normals.row(data.F(i,j)).cast<float>();


        if (invert_normals)
          meshgl.V_normals_vbo = -meshgl.V_normals_vbo;
      }

      if (meshgl.dirty & MeshGL::DIRTY_FACE)
      {
        meshgl.F_vbo.resize(data.F.rows(),3);
        for (unsigned i=0; i<data.F.rows();++i)
          meshgl.F_vbo.row(i) << i*3+0, i*3+1, i*3+2;
      }

      if (meshgl.dirty & MeshGL::DIRTY_UV)
      {
        meshgl.V_uv_vbo.resize(data.F.rows()*3,2);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_uv_vbo.row(i*3+j) =
              data.V_uv.row(per_corner_uv ?
                data.F_uv(i,j) : data.F(i,j)).cast<float>();
      }
    }
  }
  else
  {
    if (meshgl.dirty & MeshGL::DIRTY_POSITION)
    {
      per_corner(data.V,meshgl.V_vbo);
    }
    if (meshgl.dirty & MeshGL::DIRTY_AMBIENT)
    {
      per_face(data.F_material_ambient,meshgl.V_ambient_vbo);
    }
    if (meshgl.dirty & MeshGL::DIRTY_DIFFUSE)
    {
      per_face(data.F_material_diffuse,meshgl.V_diffuse_vbo);
    }
    if (meshgl.dirty & MeshGL::DIRTY_SPECULAR)
    {
      per_face(data.F_material_specular,meshgl.V_specular_vbo);
    }

    if (meshgl.dirty & MeshGL::DIRTY_NORMAL)
    {
      meshgl.V_normals_vbo.resize(data.F.rows()*3,3);
      for (unsigned i=0; i<data.F.rows();++i)
        for (unsigned j=0;j<3;++j)
          meshgl.V_normals_vbo.row(i*3+j) =
             per_corner_normals ?
               data.F_normals.row(i*3+j).cast<float>() :
               data.F_normals.row(i).cast<float>();

      if (invert_normals)
        meshgl.V_normals_vbo = -meshgl.V_normals_vbo;
    }

    if (meshgl.dirty & MeshGL::DIRTY_FACE)
    {
      meshgl.F_vbo.resize(data.F.rows(),3);
      for (unsigned i=0; i<data.F.rows();++i)
        meshgl.F_vbo.row(i) << i*3+0, i*3+1, i*3+2;
    }

    if (meshgl.dirty & MeshGL::DIRTY_UV)
    {
        meshgl.V_uv_vbo.resize(data.F.rows()*3,2);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_uv_vbo.row(i*3+j) = data.V_uv.row(per_corner_uv ? data.F_uv(i,j) : data.F(i,j)).cast<float>();
    }
  }

  if (meshgl.dirty & MeshGL::DIRTY_TEXTURE)
  {
    meshgl.tex_u = data.texture_R.rows();
    meshgl.tex_v = data.texture_R.cols();
    meshgl.tex.resize(data.texture_R.size()*4);
    for (unsigned i=0;i<data.texture_R.size();++i)
    {
      meshgl.tex(i*4+0) = data.texture_R(i);
      meshgl.tex(i*4+1) = data.texture_G(i);
      meshgl.tex(i*4+2) = data.texture_B(i);
      meshgl.tex(i*4+3) = data.texture_A(i);
    }
  }

  if (meshgl.dirty & MeshGL::DIRTY_OVERLAY_LINES)
  {
    meshgl.lines_V_vbo.resize(data.lines.rows()*2,3);
    meshgl.lines_V_colors_vbo.resize(data.lines.rows()*2,3);
    meshgl.lines_F_vbo.resize(data.lines.rows()*2,1);
    for (unsigned i=0; i<data.lines.rows();++i)
    {
      meshgl.lines_V_vbo.row(2*i+0) = data.lines.block<1, 3>(i, 0).cast<float>();
      meshgl.lines_V_vbo.row(2*i+1) = data.lines.block<1, 3>(i, 3).cast<float>();
      meshgl.lines_V_colors_vbo.row(2*i+0) = data.lines.block<1, 3>(i, 6).cast<float>();
      meshgl.lines_V_colors_vbo.row(2*i+1) = data.lines.block<1, 3>(i, 6).cast<float>();
      meshgl.lines_F_vbo(2*i+0) = 2*i+0;
      meshgl.lines_F_vbo(2*i+1) = 2*i+1;
    }
  }

  if (meshgl.dirty & MeshGL::DIRTY_OVERLAY_POINTS)
  {
    meshgl.points_V_vbo.resize(data.points.rows(),3);
    meshgl.points_V_colors_vbo.resize(data.points.rows(),3);
    meshgl.points_F_vbo.resize(data.points.rows(),1);
    for (unsigned i=0; i<data.points.rows();++i)
    {
      meshgl.points_V_vbo.row(i) = data.points.block<1, 3>(i, 0).cast<float>();
      meshgl.points_V_colors_vbo.row(i) = data.points.block<1, 3>(i, 3).cast<float>();
      meshgl.points_F_vbo(i) = i;
    }
  }
}
