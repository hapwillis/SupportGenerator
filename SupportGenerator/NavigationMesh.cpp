#include "NavigationMesh.h"

Edge::Edge(int a, int b, float len) : indexA(a), indexB(b), length(len)
{

}

EdgeContainer::EdgeContainer(int a, int b, glm::vec3 e1, glm::vec3 e2) :
	indexA(a), indexB(b)
{
	if (e1.x < e2.x) {
		negVert = e1;
		posVert = e2;
	}
	else if (e1.x == e2.x) {
		if (e1.y < e2.y) {
			negVert = e1;
			posVert = e2;
		}
		else if (e1.y == e2.y) {
			if (e1.z < e2.z) {
				negVert = e1;
				posVert = e2;
			}
			else {
				negVert = e2;
				posVert = e1;
			}
		}
		else {
			negVert = e2;
			posVert = e1;
		}
	}
	else {
		negVert = e2;
		posVert = e1;
	}

	len = glm::distance(e1, e2);
}

EdgeSet::~EdgeSet()
{
	for (EdgeContainer *e : edges) {
		delete(e);
	}
}

void EdgeSet::insert(EdgeContainer *e1)
{
	bool notFound = true;
	float key = e1->posVert.x; // TODO: this really isn't a great key.
	auto range = edgeMap.equal_range(key);

	//check edgeMap for key, then find any equal edges
	for (auto r = range.first; r != range.second; r++) {
		EdgeContainer *e2 = r->second;

		bool posEqual = glm::all(glm::equal(e1->posVert, e2->posVert));
		bool negEqual = glm::all(glm::equal(e1->negVert, e2->negVert));
		if (posEqual && negEqual) {
			notFound = false;
			break;
		}
	}

	//if not found, insert this point and set translate[tIndex] = vIndex
	if (notFound) {
		edgeMap.insert(std::pair<float, EdgeContainer*>(key, e1));
		edges.push_back(e1);
	}
}

size_t KeyFuncs::operator()(const EdgeContainer& e)const
{
	std::string s("");
	glm::vec3 n = e.negVert;
	glm::vec3 p = e.posVert;
	s.append(std::to_string(n.x));
	s.append(std::to_string(n.y));
	s.append(std::to_string(n.z));
	s.append(std::to_string(p.x));
	s.append(std::to_string(p.y));
	s.append(std::to_string(p.z));

	return std::hash<std::string>()(s);
}

bool KeyFuncs::operator()(const EdgeContainer& a, const EdgeContainer& b)const
{
	bool v1 = glm::all(glm::equal(a.negVert, b.negVert));
	bool v2 = glm::all(glm::equal(a.posVert, b.posVert));
	return v1 && v2;
}

bool operator<(const EdgeContainer& a, const EdgeContainer& b)
{
	return a.len > b.len;
}

bool operator<(const Edge& a, const Edge& b)
{
	return a.length > b.length;
}


NavigationMesh::NavigationMesh()
{

}


NavigationMesh::~NavigationMesh()
{

}

bool NavigationMesh::loadModel(Model &newModel, float dist) 
{
	if (model != &newModel || displacement != dist) {
		model = &newModel;
		displacement = dist;
		float time = glfwGetTime();
		// 27.6864 seconds to build graph
		modelGraph = new Graph(*model, dist);
		std::cout << "Time to build Graph: " << glfwGetTime() - time << std::endl;
		time = glfwGetTime();

		// 57.52 seconds to run decimateMesh
		navGraph = decimateMesh();
		std::cout << "Time to reduce mesh: " << glfwGetTime() - time << std::endl;
		time = glfwGetTime();

		navMesh = convertToMesh(navGraph);
		std::cout << "Time to convert mesh: " << glfwGetTime() - time << std::endl;
	}

	return true;
}

void NavigationMesh::initializeHeap()
{
	if(!edgeHeap.empty()) {
		edgeHeap = std::priority_queue<Edge>();
	}
	EdgeSet edges;

	int indices = modelGraph->octree->faces.size();
	std::vector<unsigned int> faces = modelGraph->octree->faces;
	for (int i = 0; i < indices; i += 3) {
		unsigned int *index = &faces[i];
		glm::vec3 a = modelGraph->nodes[*index]->position;
		glm::vec3 b = modelGraph->nodes[*(index + 1)]->position;
		glm::vec3 c = modelGraph->nodes[*(index + 2)]->position;

		edges.insert(new EdgeContainer(*index, *(index + 1), a, b));
		edges.insert(new EdgeContainer(*(index + 1), *(index + 2), b, c));
		edges.insert(new EdgeContainer(*(index + 2), *index, c, a));
	}

	// Insert the unique edges to a heap
	for (EdgeContainer *e : edges.edges) {
		edgeHeap.push(Edge(e->indexA, e->indexB, e->len));
	}
}

void NavigationMesh::getUniqueEdges(std::priority_queue<Edge> &edgeHeap)
{
	// Get initial edges from faces
	// Insert into unordered set to remove duplicates
	std::unordered_set<EdgeContainer, KeyFuncs, KeyFuncs> edges;
	int indices = modelGraph->octree->faces.size();
	std::vector<unsigned int> faces = modelGraph->octree->faces;
	for (int i = 0; i < indices; i += 3) {
		unsigned int *index = &faces[i];
		glm::vec3 a = modelGraph->nodes[*index]->position;
		glm::vec3 b = modelGraph->nodes[*(index + 1)]->position;
		glm::vec3 c = modelGraph->nodes[*(index + 2)]->position;

		edges.insert(EdgeContainer(*index, *(index + 1), a, b));
		edges.insert(EdgeContainer(*(index + 1), *(index + 2), b, c));
		edges.insert(EdgeContainer(*(index + 2), *index, c, a));
	}

	// Insert the unique edges to a heap
	for (const EdgeContainer &e : edges) {
		edgeHeap.push(Edge(e.indexA, e.indexB, e.len));
	}
}

Graph* NavigationMesh::decimateMesh()
{
	initializeHeap(); // TODO: write custom heap that uses pointers

	//for the number of verts to be removed:
	const float decimationLevel = .0001;
	const int toRemove = std::floor((1 - decimationLevel) * modelGraph->nodes.size());
	for (int i = 0; i < toRemove; i++) {
		//	pop() until you get a valid edge (ie both vertices exist)
		if (!edgeHeap.empty()) {
			Edge e = edgeHeap.top();
			edgeHeap.pop();
			while (!edgeValid(e, modelGraph) && !edgeHeap.empty()) {
				e = edgeHeap.top();
				edgeHeap.pop();
			}

			int newIndex = modelGraph->CombineNodes(e.indexA, e.indexB);
			glm::vec3 a = modelGraph->nodes[newIndex]->position;

			//	add all the new edges to the minheap
			for (int connection : modelGraph->nodes[newIndex]->connections) {
				Node *node = modelGraph->nodes[connection];
				if (node) {
					glm::vec3 b = node->position;
					float length = glm::distance(a, b);

					edgeHeap.push(Edge(newIndex, connection, length));
				}
			}
		}
	}

	Graph* g = modelGraph->ReduceFootprint();

	return g;
}

Mesh* NavigationMesh::convertToMesh(Graph *graph)
{
	std::vector<Vertex> vertices;
	vertices.reserve(graph->nodes.size());
	for (Node *node : graph->nodes) {
		vertices.push_back(Vertex(node->position, node->normal));
	}

	std::vector<unsigned int> indices;
	windFaces(graph->nodes, indices);

	return new Mesh(vertices, indices);
}

void NavigationMesh::windFaces(std::vector<Node*> &nodes, std::vector<unsigned int> &indices)
{
	//Not quite the fastest way to do this, but it's easy.
	for (int i = 0; i < nodes.size(); i++) {
		//for each node, make a list of each connection to a higher index
		for (int v2 : nodes[i]->connections) {
			if (i < v2) {
				//do the same for each of *those*
				for (int v3 : nodes[v2]->connections) {
					if (v2 < v3) {
						//check if this vertex connects to the original vertex
						for (int v1 : nodes[v3]->connections) {
							if (v1 == i) { //add face
								indices.push_back(v1);
								indices.push_back(v2);
								indices.push_back(v3);
							}
						}
					}
				}
			}
		}
	}		
}

bool NavigationMesh::edgeValid(Edge edge, Graph *graph)
{
	return graph->nodes[edge.indexA] && graph->nodes[edge.indexB];
}

void NavigationMesh::Draw(DefaultShader shader)
{
	if (navMesh) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		navMesh->Draw(shader);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
}


