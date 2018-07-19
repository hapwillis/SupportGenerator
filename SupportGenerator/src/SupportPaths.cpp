#include "SupportPaths.h"


Cylinder::Cylinder(int numSides, float supportWidth) : faces_(numSides), width_(supportWidth)
{

}

// Relocates the cylinder under consideration.
void Cylinder::updateGeometry(glm::vec3 startP, glm::vec3 endP)
{
	start_ = startP;
	end_ = endP;
	glm::vec3 sinVec = glm::cross(glm::normalize(end_ - start_), cosVec);

	vertices_.clear();
	indices_.clear();

	genVerts(sinVec, start_);
	genVerts(sinVec, end_);

	genIndices();
}

// Generates new vertices for a given cylinder position.
void Cylinder::genVerts(glm::vec3 sinVec, glm::vec3 center)
{
	float frac = pi2 / faces_;
	for (int face = 0; face < faces_; face++) {
		float theta = frac * face;
		glm::vec3 pos = center;
		pos += std::sin(theta) * sinVec + std::cos(theta) * cosVec;
	
		vertices_.push_back(pos);
	}
}

// Generates indices from the current vertices.
void Cylinder::genIndices()
{
	for (int a = 0; a < faces_ - 1; a++) {
		int b = a + 1;
		int c = a + faces_;
		int d = b + faces_;

		indices_.push_back(a);
		indices_.push_back(b);
		indices_.push_back(d);

		indices_.push_back(a);
		indices_.push_back(d);
		indices_.push_back(c);
	}
}


PFNode::PFNode(shared_ptr<Node> node) : node_(node)
{

}


PFNode::~PFNode()
{
}


Path::Path(float overhang, glm::mat4 model) 
	: maxOverhang_(overhang), transform_(model)
{

}


Path::~Path()
{
}

// Creates an internal PFNode from a Node object.
void Path::addNode(shared_ptr<Node> node)
{
  shared_ptr<PFNode> pathNode = shared_ptr<PFNode>(new PFNode(node));
	if (path_.size() > 0)
		closed_.insert(path_.back());
	path_.push_back(pathNode);
	pathNode->costToEnd_ = getCost(pathNode);
}

void Path::addNode(shared_ptr<PFNode> node) {
  if (path_.size() > 0)
    closed_.insert(path_.back());
  path_.push_back(node);
  node->costToEnd_ = getCost(node);
}

// Runs A* on this path.  
// If pathFound is true, paths will jump directly to the bed.
void Path::aStar(bool pathFound)
{
	glm::vec3 origin = path_[1]->node_->vertex_->Position;

	while (!open_.empty()) {
    shared_ptr<PFNode> current = open_.top();
		glm::vec3 currentPos = current->node_->vertex_->Position;
		if (currentPos.z <= 0.000000001f) {
			retracePath(current);
			return;
		}
		closed_.insert(current);
		open_.pop();
		if (current->node_->connections_.size() > 0) {
      shared_ptr<PFNode> pNode = current->connections_[0].lock();
			float dist = glm::distance(currentPos, pNode->node_->vertex_->Position);
			float least = pNode->costToEnd_ + dist;

			for (weak_ptr<PFNode> connection : current->connections_) { 
        shared_ptr<PFNode> c = connection.lock();
				if (closed_.count(c) == 0) {
					glm::vec3 cVect = glm::normalize(c->node_->vertex_->Position - origin);
					float angle = std::acos(glm::dot(cVect, glm::vec3(0.0f, 0.0f, -1.0f)));

					if (angle < maxOverhang_) {
						pNode = c;
						dist = glm::distance(currentPos, pNode->node_->vertex_->Position);
						float toStart = current->costToStart_ + dist;
						float temp = pNode->costToEnd_ + dist;
						if (temp < least)
							least = temp;

						if (seen_.count(c) == 0 || toStart < pNode->costToStart_) { //only add old nodes
							if (pNode->costToEnd_ >= 0.0) { //filter invalid nodes
								pNode->costToStart_ = toStart;
								pNode->parent_ = current;
								pNode->pathCost_ = pNode->costToStart_ + pNode->costToEnd_;
								open_.push(pNode);
							}
						}
					} else {
						closed_.insert(c);
					}
				}
			}
			current->costToEnd_ = least; 
			current->pathCost_ = current->costToStart_ + current->costToEnd_;
		} else {
			std::cout << "Error: unconnected node." << std::endl;
		}
	}

	if (pathFound) {
		path_.clear();
	} else {
		restartAStar(); // No direct path found- check if any nodes can see bed
	}
}

//  Generates Cylinders for each segment of the path and sets up rendering.
void Path::Geometry()
{
	if (path_.size() < 1)
		return;
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	vertices.reserve(faces_ * path_.size() * 2);
	indices.reserve(faces_ * path_.size() * 6);
	Cylinder c = Cylinder(faces_, 0.5f); //set the characteristics of the cylinder generator

	// TODO: move this logic into Cylinder; have it add new vertices and indices each time it's given a new point.
	for (int i = 1; i < (path_.size() - 1); i++) {
		c.updateGeometry(path_[i]->node_->vertex_->Position, path_[i + 1]->node_->vertex_->Position);
		glm::vec3 norm = glm::vec3(0.0f, 0.0f, 0.0f);
		
		int vOffset = vertices.size();
		for (glm::vec3 pos : c.vertices_) {
			vertices.emplace_back(pos, norm, true);
			//vertices.push_back(Vertex(pos, norm, true));
		}
		
		for (int i : c.indices_) {
			indices.push_back(i + vOffset);
		}
	}

	pathGeometry_ = shared_ptr<Mesh>(new Mesh(vertices, indices));

	for (shared_ptr<PFNode> n : path_) {
		renderpoints_.push_back(*n->node_->vertex_);
	}

	pathStep_ = 1;
	glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(10.0f);

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	glBindVertexArray(VAO);
	// load data into vertex buffers
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// A great thing about structs is that their memory layout is sequential for all its items.
	// The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
	// again translates to 3/2 floats which translates to a byte array.
	glBufferData(GL_ARRAY_BUFFER, renderpoints_.size() * sizeof(Vertex), &renderpoints_[0], GL_STATIC_DRAW);

	// set the vertex attribute pointers
	// vertex Positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Wireframe));
	glBindVertexArray(0);
}

// Draws the Cylinder geometry as well as a GL_POINT at each PFNode.  
// Also animates the travel of the path.
void Path::Draw(DefaultShader shader) {
	if (pathGeometry_) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		shader.setVec4("color", glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));
		pathGeometry_->Draw(shader);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glPolygonMode(GL_FRONT_AND_BACK, GL_POINTS);
		glBindVertexArray(VAO);
		shader.setVec4("color", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
		//glDrawArrays(GL_POINTS, 0, pathStep);
		glBindVertexArray(0);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		pathStep_++;
		if (pathStep_ == renderpoints_.size())
			pathStep_ = 1;
		
	} else {
		Geometry();
	}
}

// Calculates cost for PFNode node. 
//    Invalid nodes (eg unconnected) return -1
//    Valid nodes return admissible (underestimating) distances to the bed.
float Path::getCost(shared_ptr<PFNode> node) {
	if (node->node_->connections_.size() == 0) {
		return -1;
	}

  // TODO: actually test distance to bed and nodes where position.z = 0.0f

	return (transform_*(glm::vec4(node->node_->vertex_->Position, 1.0f))).z;
}

// Resets this path object, then attempts to pathfind by allowing nodes to skip directly to the bed.
void Path::restartAStar()
{
	closed_.clear();
	seen_.clear();
	open_ = std::priority_queue<shared_ptr<PFNode>, std::vector<shared_ptr<PFNode>>, PFNodeComparator>();
	open_.push(path_.back());
	seen_.insert(path_.back());
	for (int i = 0; i < path_.size() - 1; i++) {
		closed_.insert(path_[i]);
	}
	aStar(true);
}

// Fills a path vector by following the parentage of a valid end PFNode.
void Path::retracePath(shared_ptr<PFNode> end)
{
	std::vector<shared_ptr<PFNode>> backwards;
	backwards.push_back(end);
	while (backwards.back()->parent_) {
		backwards.push_back(backwards.back()->parent_);
	}

	while (backwards.size() > 0) {
		path_.push_back(backwards.back());
		backwards.pop_back();
	}

	closed_.clear();
	seen_.clear();
	open_ = std::priority_queue<shared_ptr<PFNode>, std::vector<shared_ptr<PFNode>>, PFNodeComparator>();
}


SupportPaths::SupportPaths( shared_ptr<Graph> modelGraph,
                            shared_ptr<Graph> navGraph,
                            shared_ptr<Octree> modelOctree,
                            shared_ptr<Octree> navOctree,
                            float max, float offset) :
	  modelGraph_(modelGraph), 
    navGraph_(navGraph),  
    modelOctree_(modelOctree),
    navOctree_(navOctree),
    maxOverhang_(max) {
	//std::cout << "Checking validity of navGraph..." << std::endl;
	//navGraph->test();

  std::vector<shared_ptr<PFNode>> nodes;
	for (shared_ptr<Node> n : navGraph_->nodes_) {
    shared_ptr<PFNode> pfn(new PFNode(n));
    pair<shared_ptr<Vertex>, shared_ptr<PFNode>> nodeToPF(n->vertex_, pfn);
    PFnodeMap_.insert(nodeToPF);

    pfn->costToEnd_ = (transform_*(glm::vec4(n->vertex_->Position, 1.0f))).z;
    nodes.push_back(pfn);
	}

  for (shared_ptr<PFNode> pathingNode : nodes) {
    for (weak_ptr<Node> n : pathingNode->node_->connections_) {
      pathingNode->connections_.push_back(PFnodeMap_.at(n.lock()->vertex_));
    }
  }

	paths_.reserve(points_.size()); 
}


SupportPaths::~SupportPaths()
{

}

// Finds paths for the given connection points.
// This handles global pathing logic:
//    Which pathfinding algorithm to use
//    Tracking failed paths
//    Handling how paths interact
//    Regrouping, descending and relaxing paths
void SupportPaths::FindPaths(std::vector<std::tuple<glm::vec3, shared_ptr<Face>>> points)
{
  for (auto tuple : points) {
    points_.push_back(std::get<0>(tuple));
  }
	int failedPaths = 0;

	for (glm::vec3 &point : points_) {
    shared_ptr<Path> path = shared_ptr<Path>(findStartNode(point));
		paths_.push_back(path);
		if (path->path_.size() > 0) {
			path->open_.push(path->path_.back());
			path->aStar(false);
		} else {
			failedPaths++;
		}

		// works best when points are sorted from lowest to highest
		// Also works best with Djikstra's instead of A*; A* doesn't search out other supports.
		if (OVERLAP_PATHS) {  
			for (shared_ptr<PFNode> node : path->path_) {
				node->costToEnd_ = 0.0f;
			}
		}
	}
	std::cout << "Failed paths: " << failedPaths << std::endl;

	regroupPaths();
}

// Generates geometry for rendering.
void SupportPaths::Geometry(int faces, float tipD)
{
	// TODO: Geometry()
}


void SupportPaths::Draw(DefaultShader shader)
{
	// TODO: completed paths should have their own geometry- render that 
	// if it exists, otherwise render the wireframes.
	for (shared_ptr<Path> path : paths_) {
		path->Draw(shader);
	}
}

// Deletes a path, given a world space vector from the camera.
void SupportPaths::DeleteSupport()
{
	// TODO: DeleteSupport
}

// Creates the first three PFnodes_ of a path:
//    1. Model connection point
//    2. A point on the proper navigation surface face
//    3. The closest Node on the navigation graph.
// Only the third node actually has connections and is valid for pathfinding.
shared_ptr<Path> SupportPaths::findStartNode(glm::vec3 point)
{
	// TODO: can just take face from ConnectionPoints.pointFaces
	shared_ptr<Face> face = modelOctree_->getNearestFace(point);
  shared_ptr<Path> path = shared_ptr<Path>(new Path(maxOverhang_, transform_));
  shared_ptr<Node> startNode = shared_ptr<Node>(new Node(-1, point, face->normal, true));
	path->addNode(startNode);

	// TODO: rayCast is broken.  Probably because the octree is unscaled.
	//glm::vec3 pointTwo = point + 2.0f * face->normal;
	glm::vec3 pointTwo = navOctree_->rayCast(point, face->normal);
  shared_ptr<Node> nodeTwo = shared_ptr<Node>(new Node(-1, pointTwo, face->normal, true));
	path->addNode(nodeTwo);

  tuple<bool, shared_ptr<Vertex>, float> getNearestVertex;
  shared_ptr<Vertex> vert = std::get<1>(navOctree_->getNearestVertex(pointTwo));
	path->addNode(PFnodeMap_.at(vert));

	//TEST:
	/*float dist = glm::distance(pointTwo, navGraph->nodes[index]->vertex.Position);
	float shortest = 10000.0f;
	int newindex = 0;
	int mindex = 0;
	for (Node *n : navGraph->nodes) {
		if (glm::distance(pointTwo, n->vertex.Position) < shortest) {
			shortest = glm::distance(pointTwo, n->vertex.Position);
			mindex = newindex;
		}
		newindex++;
	}

	if (mindex != index)
		std::cout << "Failed to find actual nearest index!" << std::endl;
	*/

	return path;
}

// Splits paths into non-overlapping paths.
void SupportPaths::regroupPaths()
{
	// TODO: regroupPaths
}

// Attempts to remove any path segments that move upwards.
// Any unfixable paths are deleted.
void SupportPaths::descendPaths() //rename to RegularizePaths
{
	// TODO: path relaxation of anything under maxAngle
}

// Relaxes paths, ie attempts to minimize the length of the path
// without intersecting the navigation surface.
//    This function is not intended to produce optimal solutions,
//    ie perfectly minimizing both path length and distance from the print.
//    It just makes reasonable approximations by straightening the line 
void SupportPaths::Relax(float degree)
{
	// TODO: Path Relaxation
	// degree is the fraction of nodes to remove- 1 means totally straight
	// Optimal relaxation is an NP-complete problem

	// Step 1 (if degree < 1.0f): Greedy algorithm
	//	While (notStuck && dist < degree * minDist)
	//		if (comboHeap.empty) :
	//			for (node : path) : add smallest combos to heap
	//		else :
	//			combine(heap.pop())

  // Alternate: value shortcuts by their distance from the closest face (shortest first)
}

// Intersects a vector of geometry objects into a single mesh.
void SupportPaths::intersectionGeometry()
{
	// TODO: intersectionGeometry
	//use: CGAL 4.12 - Polygon Mesh Processing


	// given a vector<cylinder>, project from each line segment to all the faces in the other cylinders
	// move far vertices back to the point of collision, split the collided face
}

