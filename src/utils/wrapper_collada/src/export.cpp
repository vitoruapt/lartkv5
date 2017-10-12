/**
 * @file
 * @brief A collada dom example. Definition of the class.
 */


/*
* Copyright 2006 Sony Computer Entertainment Inc.
*
* Licensed under the MIT Open Source License, for details please see license.txt or the website
* http://www.opensource.org/licenses/mit-license.php
*
*/ 
#include <dae.h>
#include <dae/daeUtils.h>
#include <dom/domCOLLADA.h>

using namespace std;
using namespace ColladaDOM150;
using namespace cdom;

// Demonstrates how to use the DOM to create a simple, textured Collada model
// and save it to disk.



template<typename T>
daeTArray<T> rawArrayToDaeArray(T rawArray[], size_t count) {
	daeTArray<T> result;
	for (size_t i = 0; i < count; i++)
		result.append(rawArray[i]);
	return result;
}

// "myGeom" --> "#myGeom"
string makeUriRef(const string& id) {
	return string("#") + id;
}

void my_addSource(daeElement* mesh,
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

	list<string> params = tokenize(paramNames, " ");
	acc->setStride(params.size());
	acc->setCount(valueCount/params.size());
	for (tokenIter iter = params.begin(); iter != params.end(); iter++) {
		//SafeAdd(acc, "param", p);
		daeElement* p = acc->add("param");
		p->setAttribute("name", iter->c_str());
		p->setAttribute("type", "float");
	}
	
	//return testResult(true);
}


void my_addInput(daeElement* triangles,
                    const string& semantic,
                    const string& srcID,
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

void my_addGeometry(daeElement* root) {

	daeElement* geomLib = root->add("library_geometries");
	daeElement* geom = geomLib->add("geometry");
	string geomID = "cubeGeom";
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

void my_addImage(daeElement* root) {
	daeElement* imageLib = root->add("library_images");
	daeElement* image = imageLib->add("image");
	image->setAttribute("id", "img");
	image->setAttribute("name", "myimage");
	image->add("init_from")->setCharData("./texture.bmp");


	return;


	//daeElement* tmp = image->add("init_from");
	daeElement* tmp = image->add("init_from");
	//image->add("init_from")->setCharData("./texture.bmp");
	std::string str("file://home/mike/workingcopy/arpua/utils/wrapper_collada/bin/texture1.png");
	//std::string str("http://dubinko.info/writing/xforms/");
	//const char* date = "file:///home/mike/workingcopy/arpua/utils/wrapper_collada/bin/texture.bmp";
	//std::string str("MAX");
	daeBool res = tmp->setCharData(str);
	printf("result is =%d",res);
}

void my_addEffect(daeElement* root) {
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

void my_addMaterial(daeElement* root) {
	//SafeAdd(root, "library_materials", materialLib);
	daeElement* materialLib = root->add("library_materials");
	//SafeAdd(materialLib, "material", material);
	daeElement* material = materialLib->add("material");
	material->setAttribute("id", "cubeMaterial");
	material->add("instance_effect")->setAttribute("url", makeUriRef("cubeEffect").c_str());

}

void my_addVisualScene(daeElement* root) {

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
