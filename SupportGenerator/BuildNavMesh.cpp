#include "Model.h"

struct Edge {
	unsigned int indexA;
	unsigned int indexB;
	float length;

	Edge(int a, int b, float len) : indexA(a), indexB(b), length(len)
	{

	}
};

struct EdgeContainer {
	unsigned int indexA;
	unsigned int indexB;
	glm::vec3 negVert;
	glm::vec3 posVert;
	float len;

	EdgeContainer(int a, int b, glm::vec3 e1, glm::vec3 e2) : 
		indexA(a), indexB(b)
	{
		if (e1.x < e2.x) {
			negVert = e1;
			posVert = e2;
		} else if (e1.x == e2.x) {
			if (e1.y < e2.y) {
				negVert = e1;
				posVert = e2;
			} else if (e1.y == e2.y) {
				if (e1.z < e2.z) {
					negVert = e1;
					posVert = e2;
				} else {
					negVert = e2;
					posVert = e1;
				}
			} else {
				negVert = e2;
				posVert = e1;
			}
		} else {
			negVert = e2;
			posVert = e1;
		}

		len = glm::distance(e1, e2);
	}
};

struct KeyFuncs
{
	size_t operator()(const EdgeContainer& e)const
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

	bool operator()(const EdgeContainer& a, const EdgeContainer& b)const
	{
		bool v1 = glm::all(glm::equal(a.negVert, b.negVert));
		bool v2 = glm::all(glm::equal(a.posVert, b.posVert));
		return v1 && v2;
	}
};

bool operator<(const EdgeContainer& a, const EdgeContainer& b)
{
	return a.len < b.len;
}

bool operator<(const Edge& a, const Edge& b)
{
	return a.length < b.length;
}

std::vector<glm::vec3> BuildNavGraph(Model &model, float dist)
{
	Graph modelGraph(model, dist);

	modelGraph = decimateMesh(modelGraph);

	// Better to convert these to an indexed mesh, but lines are easy
	return convertToLines(modelGraph);
}

void initializeHeap(Graph &modelGraph, std::priority_queue<Edge> &edgeHeap)
{
	std::unordered_set<EdgeContainer, KeyFuncs, KeyFuncs> edges;

	// Get initial edges from faces
	// Insert into unordered set to remove duplicates
	int indices = modelGraph.modelRef->octree.faces.size();
	std::vector<unsigned int> faces = modelGraph.modelRef->octree.faces;
	for (int i = 0; i < indices; i += 3) {
		glm::vec3 a = modelGraph.nodes[faces[i]]->position;
		glm::vec3 b = modelGraph.nodes[faces[i + 1]]->position;
		glm::vec3 c = modelGraph.nodes[faces[i + 2]]->position;
		edges.insert(EdgeContainer(faces[i], faces[i + 1], a, b));
		edges.insert(EdgeContainer(faces[i + 1], faces[i + 2], b, c));
		edges.insert(EdgeContainer(faces[i + 2], faces[i], c, a));
	}

	// Insert the unique edges to a heap
	for (const EdgeContainer &e : edges) {
		edgeHeap.push(Edge(e.indexA, e.indexB, e.len));
	}
}

bool edgeValid(Edge edge, Graph modelGraph)
{
	return modelGraph.nodes[edge.indexA] && modelGraph.nodes[edge.indexA];
}

Graph decimateMesh(Graph modelGraph)
{
	// TODO: write custom heap that uses pointers
	std::priority_queue<Edge> edgeHeap;
	initializeHeap(modelGraph, edgeHeap);

	std::vector<Node*> nodes = modelGraph.nodes;

	//for the number of verts to be removed:
	const float decimationLevel = .0001;
	const int toRemove = std::floor((1 - decimationLevel) * modelGraph.nodes.size());
	for (int i = 0; i < toRemove; i++) {
		//	pop() until you get a valid edge (ie both vertices exist)
		if (!edgeHeap.empty()) {
			Edge e = edgeHeap.top(); 
			edgeHeap.pop();
			while(!edgeValid(e, modelGraph) && !edgeHeap.empty()) {
				e = edgeHeap.top();
				edgeHeap.pop();
			}

			int newIndex = modelGraph.CombineNodes(e.indexA, e.indexB);
			glm::vec3 a = modelGraph.nodes[newIndex]->position;

			//	add all the new edges to the minheap
			for (int connection : modelGraph.nodes[newIndex]->connections) {
				glm::vec3 b = modelGraph.nodes[connection]->position;
				float length = glm::distance(a, b);

				edgeHeap.push(Edge(newIndex, connection, length));
			}
		}
	}
	
	return modelGraph;
}

std::vector<glm::vec3> convertToLines(Graph &modelGraph)
{
	modelGraph.ReduceFootprint();
	std::vector<std::unordered_set<int>> lines;
	lines.reserve(modelGraph.nodes.size());

	for (int i = 0; i < modelGraph.nodes.size(); i++) {
		Node *node = modelGraph.nodes[i];
		for (int j : node->connections) {
			if (i < j) {
				lines[i].insert(j);
			} else {
				lines[j].insert(i);
			}
		}
	}

	std::vector<glm::vec3> lineVertices;
	for (int i = 0; i < lines.size(); i++) {
		for (int j : lines[i]) {
			lineVertices.push_back(modelGraph.nodes[i]->position);
			lineVertices.push_back(modelGraph.nodes[j]->position);
		}
	}

	return lineVertices;
}
