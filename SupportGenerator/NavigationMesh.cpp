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

NavigationMesh::NavigationMesh(Model &newModel, float offset, float width)
{
	model = &newModel;
	displacement = offset + (width * 0.5f);
	supportWidth = width;
	//double time = glfwGetTime();
	// 27.6864 seconds to build graph
	graph = new Graph(*model);
	//std::cout << "Time to build Graph: " << glfwGetTime() - time << std::endl;
}

NavigationMesh::~NavigationMesh()
{
	
}

Graph * NavigationMesh::getSimpleGraph(float offset, float width)
{
	//float time = glfwGetTime();
	// 57.52 seconds to run decimateMesh
	navGraph = decimateMesh();
	//std::cout << "Time to reduce mesh: " << glfwGetTime() - time << std::endl;

	return navGraph;
}

void NavigationMesh::initializeHeap(std::priority_queue<Edge> &edgeHeap) //Appears to not be handling edges properly
{
	if(!edgeHeap.empty()) {
		edgeHeap = std::priority_queue<Edge>();
	}
	EdgeSet edges;

	for (unsigned int i = 0; i < graph->faceVector.size(); i += 3) {
		Face *face = graph->faceVector[i];
		glm::vec3 a = face->vertex1;
		glm::vec3 b = face->vertex2;
		glm::vec3 c = face->vertex3;

		edges.insert(new EdgeContainer(face->index1, face->index2, a, b));
		edges.insert(new EdgeContainer(face->index2, face->index3, b, c));
		edges.insert(new EdgeContainer(face->index3, face->index1, c, a));
	}

	// Insert the unique edges to a heap
	for (EdgeContainer *e : edges.edges) {
		edgeHeap.push(Edge(e->indexA, e->indexB, e->len));
	}
}

Graph* NavigationMesh::decimateMesh()
{
	// TODO: rewrite this so it doesn't corrupt the graph nodes.
	std::priority_queue<Edge> edgeHeap;
	// TODO: heap appears to be broken
	initializeHeap(edgeHeap); // TODO: write custom heap that uses pointers

	float minLength = supportWidth * 0.25f;
	float smallestEdge = minLength - 1.0f; //TODO: remove configuration, set to sane defaults
	while (!edgeHeap.empty() && smallestEdge < minLength) {
		//	pop() until you get a valid edge (ie both vertices exist)
		Edge e = edgeHeap.top();
		edgeHeap.pop();
		while (!edgeValid(e, graph) && !edgeHeap.empty()) {
			e = edgeHeap.top();
			edgeHeap.pop();
		}
		smallestEdge = e.length;

		int newIndex = graph->CombineNodes(e.indexA, e.indexB);
		glm::vec3 a = graph->nodes[newIndex]->vertex.Position;

		//	add all the new edges to the minheap
		for (int connection : graph->nodes[newIndex]->connections) {
			Node *node = graph->nodes[connection];
			if (node) {
				glm::vec3 b = node->vertex.Position;
				float length = glm::distance(a, b);
		
				edgeHeap.push(Edge(newIndex, connection, length));
			}
		}
	} 

	return graph->ReduceFootprint();
}

Mesh* NavigationMesh::convertToMesh(Graph *graph, float offset)
{
	//float time = glfwGetTime();
	graph->scale(offset);
	graph->recalculateNormalsFromFaces();
	std::vector<Vertex> vertices;
	vertices.reserve(graph->nodes.size());

	for (Node *node : graph->nodes) {
		vertices.push_back(Vertex(node->vertex.Position, node->vertex.Normal, 1.0));
	}

	std::vector<unsigned int> indices;
	facesToIndices(graph, indices);

	mesh = new Mesh(vertices, indices);
	//std::cout << "Time to convert mesh: " << glfwGetTime() - time << std::endl;
	return mesh;
}

void NavigationMesh::facesToIndices(Graph *graph, std::vector<unsigned int> &indices)
{
	//std::cout << "number of faces: " << graph->faceVector.size() << std::endl;
	//std::cout << "number of vertices: " << graph->nodes.size() << std::endl;

	//It's gotta be this, right?  This isn't creating all the faces?
	int invalid = 0;
	int size = graph->nodes.size();
	for (int i = 0; i < size; i++) {
		Node *node = graph->nodes[i];
		for (int f : node->faces) {
			if (f != -1) { //TODO: remove this once missing faces are filtered out. 
				Face *face = graph->faceVector[f];
				indices.push_back(face->index1);
				indices.push_back(face->index2);
				indices.push_back(face->index3);
			} else {
				invalid++;
			}
		}
	}

	//std::cout << "Corrupted face indices: " << invalid << std::endl;
}

bool NavigationMesh::edgeValid(Edge edge, Graph *graph)
{
	return graph->nodes[edge.indexA] && graph->nodes[edge.indexB];
}

void NavigationMesh::Draw(DefaultShader shader)
{
	if (mesh) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		mesh->Draw(shader);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
}


