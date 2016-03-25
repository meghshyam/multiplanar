/*
 *   File Name: makeBoundingRects.hpp
 *     Project: Multiple Plane JLinkage
 *  Created on: 22-Mar-2016
 *      Author: Sona Praneeth Akula
 *     Details:
 *   TodoNotes: TODO
 */

#ifndef MAKEBOUNDINGRECTS_HPP_
#define MAKEBOUNDINGRECTS_HPP_


void orderPlanePointsByCentroids(
		const vector<Point3d> &projectionOf3DPoints,
		const vector< vector<double> > &planeParameters,
		const map<LLI, pair<LLI, LLI> > &planeIndexBounds,
		vector<Point3d> &sortedProjectionsOf3DPoints,
		vector< vector<double> > &sortedPlaneParameters,
		map<LLI, pair<LLI, LLI> > &sortedPlaneIndexBounds );


void getBoundingBoxCoordinates (
		const vector<Point3d> &sortedProjectionOf3DPoints,
		const vector< vector<double> > &sortedPlaneParameters,
		const map<LLI, pair<LLI, LLI> > &sortedPlaneIndexBounds,
		vector< vector<Point3d> > &boundingBoxPoints );


void getContinuousBoundingBox (
		const vector< vector<Point3d> > &boundingBoxPoints,
		const vector< vector<double> > &sortedPlaneParameters,
		vector< vector<Point3d> > &continuousBoundingBoxPoints);

#endif /* MAKEBOUNDINGRECTS_HPP_ */