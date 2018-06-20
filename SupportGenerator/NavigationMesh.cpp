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
	return a.len < b.len;
}

bool operator<(const Edge& a, const Edge& b)
{
	return a.length < b.length;
}


NavigationMesh::NavigationMesh()
{
	setupRender();
}


NavigationMesh::~NavigationMesh()
{

}

bool NavigationMesh::loadModel(Model &newModel, float dist) 
{
	model = &newModel;
	displacement = dist;
	modelGraph = new Graph(*model, dist);

	navGraph = decimateMesh();

	// Better to convert these to an indexed mesh, but lines are easy
	convertToMesh(navGraph);

	return true;
}

void NavigationMesh::initializeHeap()
{
	if(!edgeHeap.empty()) {
		edgeHeap = std::priority_queue<Edge>();
	}

	// Get initial edges from faces
	// Insert into unordered set to remove duplicates
	std::unordered_set<EdgeContainer, KeyFuncs, KeyFuncs> edges;
	int indices = modelGraph->octree->faces.size();
	std::vector<unsigned int> faces = modelGraph->octree->faces;
	for (int i = 0; i < indices; i += 3) {
		// probable overoptimization here
		unsigned int *index = &faces[i];
		Node **n = &modelGraph->nodes[*index];
		glm::vec3 a = (*n)->position;
		glm::vec3 b = (*(n + 1))->position;
		glm::vec3 c = (*(n + 2))->position;
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
				glm::vec3 b = modelGraph->nodes[connection]->position;
				float length = glm::distance(a, b);

				edgeHeap.push(Edge(newIndex, connection, length));
			}
		}
	}

	return modelGraph->ReduceFootprint();
}

Mesh* NavigationMesh::convertToMesh(Graph *graph)
{
	std::vector<std::unordered_set<int>> lines;
	lines.reserve(graph->nodes.size());

	for (int i = 0; i < graph->nodes.size(); i++) {
		Node *node = graph->nodes[i];
		for (int j : node->connections) {
			if (i < j) {
				lines[i].insert(j);
			}
			else {
				lines[j].insert(i);
			}
		}
	}

	std::vector<glm::vec3> lineVerts;
	for (int i = 0; i < lines.size(); i++) {
		for (int j : lines[i]) {
			lineVerts.push_back(graph->nodes[i]->position);
			lineVerts.push_back(graph->nodes[j]->position);
		}
	}

	return lineVerts;
}

bool NavigationMesh::edgeValid(Edge edge, Graph *graph)
{
	return graph->nodes[edge.indexA] && graph->nodes[edge.indexA];
}

void NavigationMesh::Draw(DefaultShader shader)
{
	navMesh->Draw(shader);
}


