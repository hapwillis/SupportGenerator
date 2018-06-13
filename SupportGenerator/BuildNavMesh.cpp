#include "Model.h"

struct Edge {
	glm::vec3 negVert;
	glm::vec3 posVert;
	float len;

	Edge(glm::vec3 e1, glm::vec3 e2)
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
	size_t operator()(const Edge& e)const
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

	bool operator()(const Edge& a, const Edge& b)const
	{
		bool v1 = glm::all(glm::equal(a.negVert, b.negVert));
		bool v2 = glm::all(glm::equal(a.posVert, b.posVert));
		return v1 && v2;
	}
};

bool operator<(const Edge& a, const Edge& b)
{
	return a.len < b.len;
}

void BuildNavMesh(Model model, float dist)
{
	Mesh navMesh = InitNavMesh(model, dist);

	navMesh = decimateMesh(navMesh);
}

Mesh InitNavMesh(Model model, float dist) 
{
	std::vector<Vertex> vertices;
	std::vector<unsigned int> faces;
	vertices.reserve(model.octree.vertices.size());

	//TODO: remove vertices at z = 0

	int i = 0;
	for (int index : model.octree.faces) {
		faces[i] = index;
		i++;
	}

	i = 0;
	for (Vertex *v : model.octree.vertices) {
		glm::vec3 pos = v->Position;
		glm::vec3 norm = v->Normal;
		pos += (norm * dist);
		vertices[i] = Vertex(pos, norm);
		i++;
	}

	return Mesh(vertices, faces);
}

Mesh decimateMesh(Mesh navMesh)
{
	std::unordered_set<Edge, KeyFuncs, KeyFuncs> edges;

	// construct Graph
	int indices = navMesh.indices.size();
	for (int i = 0; i < indices; i += 3) {
		glm::vec3 v1 = navMesh.vertices[i].Position;
		glm::vec3 v2 = navMesh.vertices[i + 1].Position;
		glm::vec3 v3 = navMesh.vertices[i + 2].Position;
		edges.insert(Edge(v1, v2));
		edges.insert(Edge(v2, v3));
		edges.insert(Edge(v3, v1));
	}

	// Insert all edges in graph to the heap
	std::priority_queue <Edge> edgeHeap;
	for (const Edge &edge : edges) {
		edgeHeap.push(edge);
	}

	//for the number of verts to be removed:
	//	edgeHeap.pop() until you get a valid edge

	//	call graph.combine()
	//	add all the new edges to the heap
	// delete the old two vertices
}
