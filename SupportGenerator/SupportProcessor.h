#pragma once

#include <glad/glad.h>

#include <Model.h>
#include <Octree.h>
#include <Graph.h>
#include <NavigationMesh.h>
#include <ConnectionPoints.h>
#include <SupportPaths.h>

#include <vector>

using std::vector;
using std::tuple;
using std::shared_ptr;
using std::unordered_multimap;
using glm::vec3;

// Interface for adding supports to a model.
class SupportProcessor {
public:
  float width_ = 0.1;

  vector<shared_ptr<Vertex>> modelVertices_;
  vector<int> modelIndices_;
  shared_ptr<Octree> modelOctree_;

  vector<tuple<vec3, vec3>> supportOrigins_;

  shared_ptr<Graph> navigationGraph_;
  shared_ptr<Octree> navigationOctree_;

  SupportProcessor();
  SupportProcessor(const SupportProcessor& that);
  SupportProcessor(SupportProcessor&& that);
  ~SupportProcessor();

  void Draw();
  int loadModel(Model model);
  void generateOrigins(float maxAngle);
  void generateSupports(float offset, float width);

private: 

  void setUpRendering();
  Model makeModel();

  std::vector<int> constructUniqueVertices(int size);
  int checkMultimap(Vertex& v, unordered_multimap<float, int>& vertexMap);
  void constructUniqueFaces(int size, vector<int>& translate);
  bool pointsEqual(vec3 p, vec3 q);
};

