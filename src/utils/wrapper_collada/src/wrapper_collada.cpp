/**************************************************************************************************
 Software License Agreement (BSD License)

 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:

  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/**
 * @addtogroup wrapper_collada 
 *@{
 */
#ifndef _wrapper_collada_CPP_
#define _wrapper_collada_CPP_

/**
 * @file wrapper_collada.cpp
 * @brief the wrapper collada class methods definitions
 * @author Miguel Armando Riem de Oliveira
 * @version 0.0
 * @date 2011-09-15
 */

//####################################################################
//// Defines:
////####################################################################

//####################################################################
//// Includes:
////####################################################################
#include "wrapper_collada/wrapper_collada.h"


void wrapper_collada::add_polygon_fixed_color(std::string polygon_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  pcl::PointCloud<pcl::PointXYZ>::Ptr normal, float r, float g, float b, float a) 
{
	//Add geometry
	daeElement* geomLib = root->add("library_geometries");
	daeElement* geom = geomLib->add("geometry");
	geom->setAttribute("id", (polygon_name + "-geom_id").c_str());
	daeElement* mesh = geom->add("mesh");

	// Add the position data (vertices)
	daeElement* source = mesh->add("source");
	source->setAttribute("id", (geom->getAttribute("id") + "-positions").c_str());
	domFloat_array* fa = daeSafeCast<domFloat_array>(source->add("float_array"));
	fa->setId((source->getAttribute("id") + "-array").c_str());
	fa->setCount((int)cloud->points.size()*3);
	fa->getValue() = pcl_pointcloud_tp_daearray(cloud);
	domAccessor* acc = daeSafeCast<domAccessor>(source->add("technique_common accessor"));
	acc->setSource(makeUriRef(fa->getId()).c_str());
	std::list<std::string> params = cdom::tokenize("X Y Z", " ");
	acc->setStride(params.size());
	acc->setCount((int)cloud->points.size());
	for (cdom::tokenIter iter = params.begin(); iter != params.end(); iter++) 
	{
		daeElement* p = acc->add("param");
		p->setAttribute("name", iter->c_str());
		p->setAttribute("type", "float");
	}

	// Add the normal to the polygon
	daeElement* source1 = mesh->add("source");
	source1->setAttribute("id", (geom->getAttribute("id") + "-normals").c_str());
	domFloat_array* fa1 = daeSafeCast<domFloat_array>(source1->add("float_array"));
	fa1->setId((source1->getAttribute("id") + "-array").c_str());
	fa1->setCount((int)normal->points.size()*3); //just one normal
	fa1->getValue() = pcl_pointcloud_tp_daearray(normal);
	domAccessor* acc1 = daeSafeCast<domAccessor>(source1->add("technique_common accessor"));
	acc1->setSource(makeUriRef(fa1->getId()).c_str());
	std::list<std::string> params1 = cdom::tokenize("X Y Z", " ");
	acc1->setStride(params1.size());
	acc1->setCount((int)normal->points.size());
	for (cdom::tokenIter iter = params1.begin(); iter != params1.end(); iter++) 
	{
		daeElement* p = acc1->add("param");
		p->setAttribute("name", iter->c_str());
		p->setAttribute("type", "float");
	}

	// Add the <vertices> index
	daeElement* vertices = mesh->add("vertices");
	vertices->setAttribute("id", (geom->getAttribute("id") + "-vertices").c_str());
	daeElement* verticesInput = vertices->add("input");
	verticesInput->setAttribute("semantic", "POSITION");
	verticesInput->setAttribute("source", makeUriRef(geom->getAttribute("id") + "-positions").c_str());

	//Now finally add the polygon
	domPolygons* polygons = daeSafeCast<domPolygons>(mesh->add("polygons"));
	polygons->setCount(1); //just one polygon
	polygons->setMaterial("mtl"); //just one polygon
	my_addInput(polygons, "VERTEX",   makeUriRef(geom->getAttribute("id") + "-vertices"), 0);
	my_addInput(polygons, "NORMAL",   makeUriRef(geom->getAttribute("id") + "-normals"), 1);
	domP* p = daeSafeCast<domP>(polygons->add("p"));
	daeTArray<long long unsigned int> Arr;
	for (long long unsigned int i = 0; i < (long long unsigned int)cloud->points.size(); i++)
	{
		Arr.append(i); //append the vertex
		Arr.append(0); //append the normal
	}
	p->getValue() = Arr;

	char str[1024];
	sprintf(str,"%f %f %f %f",r,g,b,a);

	daeElement* effectLib = root->add("library_effects");
	daeElement* effect = effectLib->add("effect");
	effect->setAttribute("id", (polygon_name + "-effect").c_str());
	daeElement* profile = effect->add("profile_COMMON");
	daeElement* technique = profile->add("technique");
	technique->setAttribute("sid", "phong1");
	daeElement* phong = technique->add("phong");
	daeElement* emission = phong->add("emission color");
	emission->setCharData(str);
	daeElement* ambient = phong->add("ambient color");
	ambient->setCharData(str);
	daeElement* diffuse = phong->add("diffuse color");
	diffuse->setCharData(str);
	daeElement* specular = phong->add("specular color");
	specular->setCharData(str);
	daeElement* shininess = phong->add("shininess float");
	shininess->setCharData("2.0");
	daeElement* reflective = phong->add("reflective color");
	reflective->setCharData(str);
	daeElement* reflectivity = phong->add("reflectivity float");
	reflectivity->setCharData("0.5");
	daeElement* transparent = phong->add("transparent color");
	transparent->setCharData(str);
	daeElement* transparency = phong->add("transparency float");
	transparency->setCharData("1.0");

	//Material
	daeElement* materialLib = root->add("library_materials");
	daeElement* material = materialLib->add("material");
	material->setAttribute("id", (polygon_name + "-material").c_str());
	daeElement* instance_effect = material->add("instance_effect");
	instance_effect->setAttribute("url", ("#" + polygon_name + "-effect").c_str());


	//Image
	my_addImage(root);

	// Add a <node> with a simple transformation
	daeElement* node = visualScene->add("node");
	node->setAttribute("id", ("#" + polygon_name + "-node").c_str());
	node->add("scale")->setCharData("1 1 1");

	// Instantiate the <geometry>
	daeElement* instanceGeom = node->add("instance_geometry");
	instanceGeom->setAttribute("url", makeUriRef((polygon_name + "-geom_id").c_str()).c_str());

	// Bind material parameters
	daeElement* instanceMaterial = instanceGeom->add("bind_material technique_common instance_material");
	instanceMaterial->setAttribute("symbol", "mtl");
	instanceMaterial->setAttribute("target", makeUriRef((polygon_name + "-material").c_str()).c_str());





}

// ---------------------------------------
// ------- Private methods
// ---------------------------------------

template<typename T_wc>
daeTArray<T_wc> wrapper_collada::rawArrayToDaeArray(T_wc rawArray[], size_t count) {
	daeTArray<T_wc> result;
	for (size_t i = 0; i < count; i++)
		result.append(rawArray[i]);
	return result;
}

std::string wrapper_collada::makeUriRef(const std::string& id) {
	return std::string("#") + id;
}

void wrapper_collada::my_addSource(daeElement* mesh,
		const std::string& srcID,
		const std::string& paramNames,
		domFloat values[],
		int valueCount) {

	daeElement* src = mesh->add("source");
	//SafeAdd(mesh, "source", src);

	src->setAttribute("id", srcID.c_str());

	domFloat_array* fa = daeSafeCast<domFloat_array>(src->add("float_array"));
	//CheckResult(fa);
	fa->setId((src->getAttribute("id") + "-array").c_str());
	fa->setCount(valueCount);
	fa->getValue() = rawArrayToDaeArray(values, valueCount);

	domAccessor* acc = daeSafeCast<domAccessor>(src->add("technique_common accessor"));
	//CheckResult(acc);
	acc->setSource(makeUriRef(fa->getId()).c_str());

	std::list<std::string> params = cdom::tokenize(paramNames, " ");
	acc->setStride(params.size());
	acc->setCount(valueCount/params.size());
	for (cdom::tokenIter iter = params.begin(); iter != params.end(); iter++) {
		//SafeAdd(acc, "param", p);
		daeElement* p = acc->add("param");
		p->setAttribute("name", iter->c_str());
		p->setAttribute("type", "float");
	}

	//return testResult(true);
}

daeTArray<double> wrapper_collada::pcl_pointcloud_tp_daearray(pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
{
	daeTArray<double> Arr;

	for (size_t i = 0; i < pc->points.size(); i++)
	{
		Arr.append(pc->points[i].x);	
		Arr.append(pc->points[i].y);	
		Arr.append(pc->points[i].z);	
	}

	return Arr;
}

void wrapper_collada::my_addInput(daeElement* triangles,
		const std::string& semantic,
		const std::string& srcID,
		int offset) {
	domInput_local_offset* input = daeSafeCast<domInput_local_offset>(triangles->add("input"));
	//CheckResult(input);
	input->setSemantic(semantic.c_str());
	input->setOffset(offset);
	domUrifragment source(*triangles->getDAE(), srcID);
	input->setSource(source);
	if (semantic == "TEXCOORD")
		input->setSet(0);
	//return testResult(true);
}

void wrapper_collada::my_addGeometry(daeElement* root) {

	daeElement* geomLib = root->add("library_geometries");
	daeElement* geom = geomLib->add("geometry");
	std::string geomID = "cubeGeom";
	geom->setAttribute("id", geomID.c_str());
	daeElement* mesh = geom->add("mesh");

	// Add the position data
	domFloat posArray[] = { -10, -10, -10,
		-10, -10,  10,
		-10,  10, -10,
		-10,  10,  10,
		10, -10, -10,
		10, -10,  10,
		10,  10, -10,
		10,  10,  10 };
	int count = sizeof(posArray)/sizeof(posArray[0]);
	my_addSource(mesh, geomID + "-positions", "X Y Z", posArray, count);

	// Add the normal data
	domFloat normalArray[] = {  1,  0,  0,
		-1,  0,  0,
		0,  1,  0,
		0, -1,  0,
		0,  0,  1,
		0,  0, -1 };
	count = sizeof(normalArray)/sizeof(normalArray[0]);
	my_addSource(mesh, geomID + "-normals", "X Y Z", normalArray, count);

	// Add the tex coord data
	domFloat uvArray[] = { 0, 0,
		0, 1,
		1, 0,
		1, 1 };
	count = sizeof(uvArray)/sizeof(uvArray[0]);
	my_addSource(mesh, geomID + "-uv", "S T", uvArray, count);

	// Add the <vertices> element
	daeElement* vertices = mesh->add("vertices");
	vertices->setAttribute("id", (geomID + "-vertices").c_str());
	daeElement* verticesInput = vertices->add("input");
	verticesInput->setAttribute("semantic", "POSITION");
	verticesInput->setAttribute("source", makeUriRef(geomID + "-positions").c_str());

	// Add the <triangles> element.
	// Each line is one triangle.
	domUint	indices[] = {	0, 1, 0,   1, 1, 1,   2, 1, 2,
		1, 1, 1,   3, 1, 3,   2, 1, 2,
		0, 2, 0,   4, 2, 1,   1, 2, 2,
		4, 2, 1,   5, 2, 3,   1, 2, 2,
		1, 4, 0,   5, 4, 1,   3, 4, 2,
		5, 4, 1,   7, 4, 3,   3, 4, 2,
		5, 0, 0,   4, 0, 1,   7, 0, 2,
		4, 0, 1,   6, 0, 3,   7, 0, 2,
		4, 5, 0,   0, 5, 1,   6, 5, 2,
		0, 5, 1,   2, 5, 3,   6, 5, 2,
		3, 3, 0,   7, 3, 1,   2, 3, 2,
		7, 3, 1,   6, 3, 3,   2, 3, 2 };
	count = sizeof(indices)/sizeof(indices[0]);

	domTriangles* triangles = daeSafeCast<domTriangles>(mesh->add("triangles"));
	triangles->setCount(count/(3*3)); // 3 indices per vertex, 3 vertices per triangle
	triangles->setMaterial("mtl");

	my_addInput(triangles, "VERTEX",   geomID + "-vertices", 0);
	my_addInput(triangles, "NORMAL",   geomID + "-normals",  1);
	my_addInput(triangles, "TEXCOORD", geomID + "-uv",       2);

	domP* p = daeSafeCast<domP>(triangles->add("p"));
	p->getValue() = rawArrayToDaeArray(indices, count);

}

void wrapper_collada::my_addImage(daeElement* root) {
	daeElement* imageLib = root->add("library_images");
	daeElement* image = imageLib->add("image");
	image->setAttribute("id", "img");
	image->setAttribute("name", "myimage");
	image->setAttribute("format","png");
	image->setAttribute("height","375");
	image->setAttribute("width","500");
	image->setAttribute("depth","3");

	image->add("init_from")->setCharData("../texture.bmp");




	//daeElement* tmp = image->add("init_from");
	//daeElement* tmp = image->add("init_from");
	//image->add("init_from")->setCharData("./texture.bmp");
	std::string str("file://home/mike/workingcopy/arpua/utils/wrapper_collada/bin/texture1.png");
	//std::string str("http://dubinko.info/writing/xforms/");
	//const char* date = "file:///home/mike/workingcopy/arpua/utils/wrapper_collada/bin/texture.bmp";
	//std::string str("MAX");
	//daeBool res = tmp->setCharData(str);
	//tmp->se
	//daeDocument image_file()
	daeBool res1;

	//dae->

	//image->setAttribute("name", "myimage");


	return;


	////daeElement* tmp = image->add("init_from");
	//daeElement* tmp = image->add("init_from");
	////image->add("init_from")->setCharData("./texture.bmp");
	//std::string str("file://home/mike/workingcopy/arpua/utils/wrapper_collada/bin/texture1.png");
	////std::string str("http://dubinko.info/writing/xforms/");
	////const char* date = "file:///home/mike/workingcopy/arpua/utils/wrapper_collada/bin/texture.bmp";
	////std::string str("MAX");
	//daeBool res = tmp->setCharData(str);
	//printf("result is =%d",res);


	////tmp->setURI
   //res1	= tmp->hasCharData();
	//printf("init_from can have char data =%d\n",res1);
	//ROS_INFO("Setting atribute");
	////image->setAttribute("init_from",str.c_str());
	////tmp->setDocument(str.c_str());
	//ROS_INFO("Setting atribute done");
	////printf("result is =%d",res);
}

void wrapper_collada::my_addEffect(daeElement* root) {
	daeElement* effectLib = root->add("library_effects");
	daeElement* effect = effectLib->add("effect");
	effect->setAttribute("id", "cubeEffect");
	daeElement* profile = effect->add("profile_COMMON");

	// Add a <sampler2D>
	daeElement* newparam = profile->add("newparam");
	newparam->setAttribute("sid", "sampler");


	daeElement* sampler = newparam->add("sampler2D");
	daeSafeCast<domInstance_image>(sampler->add("instance_image"))->setUrl("#img");
	sampler->add("minfilter")->setCharData("LINEAR");
	sampler->add("magfilter")->setCharData("LINEAR");

	daeElement* technique = profile->add("technique");
	technique->setAttribute("sid", "common");
	daeElement* texture = technique->add("phong diffuse texture");
	texture->setAttribute("texture", "sampler");
	texture->setAttribute("texcoord", "uv0");
}

void wrapper_collada::my_addMaterial(daeElement* root) {
	//SafeAdd(root, "library_materials", materialLib);
	daeElement* materialLib = root->add("library_materials");
	//SafeAdd(materialLib, "material", material);
	daeElement* material = materialLib->add("material");
	material->setAttribute("id", "cubeMaterial");
	material->add("instance_effect")->setAttribute("url", makeUriRef("cubeEffect").c_str());

}

void wrapper_collada::my_addVisualScene(daeElement* root) {

	daeElement* visualSceneLib = root->add("library_visual_scenes");
	daeElement* visualScene = visualSceneLib->add("visual_scene");
	visualScene->setAttribute("id", "cubeScene");

	// Add a <node> with a simple transformation
	daeElement* node = visualScene->add("node");
	node->setAttribute("id", "cubeNode");
	node->add("rotate")->setCharData("1 0 0 45");
	node->add("translate")->setCharData("0 10 0");

	// Instantiate the <geometry>
	daeElement* instanceGeom = node->add("instance_geometry");
	instanceGeom->setAttribute("url", makeUriRef("cubeGeom").c_str());

	// Bind material parameters
	daeElement* instanceMaterial = instanceGeom->add("bind_material technique_common instance_material");
	instanceMaterial->setAttribute("symbol", "mtl");
	instanceMaterial->setAttribute("target", makeUriRef("cubeMaterial").c_str());

	daeElement* bindVertexInput = instanceMaterial->add("bind_vertex_input");
	bindVertexInput->setAttribute("semantic", "uv0");
	bindVertexInput->setAttribute("input_semantic", "TEXCOORD");
	bindVertexInput->setAttribute("input_set", "0");

	// Add a <scene>
	root->add("scene instance_visual_scene")->setAttribute("url", makeUriRef("cubeScene").c_str());

}


#endif
/**
 *@}
 */
