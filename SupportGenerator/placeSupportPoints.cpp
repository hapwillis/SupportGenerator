#include "Octree.h"

void InitializeSupportPoints(Model *model) 
{
	// model shouldn't have an octree.
	//model.octree = Octree(model.meshes, model.vertices, model.faces);
}

std::vector<glm::vec3> PlaceSupportPoints(Model *model)
{
	InitializeSupportPoints(model);
	return std::vector<glm::vec3>();
}