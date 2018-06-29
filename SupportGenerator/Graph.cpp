#include "Graph.h"

Graph::Graph(const Model &model)
{
	//float time = glfwGetTime();
	//0.131 seconds to remove vertices
	ConcatenateModelMeshes(model);
	//std::cout << "Time to simplify Mesh: " << glfwGetTime() - time << std::endl;

	// Populate every node with its faces and connections:
	populateConnections();

	//float time = glfwGetTime();
	//WARNING: recalculating normals breaks manifold.
	//recalculateNormalsFromFaces();
	//std::cout << "Time to recalculate normals: " << glfwGetTime() - time << std::endl;

	// TODO: start thread to create octree
}

Graph::Graph(std::vector<Node*> newNodes, std::vector<Face*> faces)
	: nodes(newNodes), faceVector(faces)
{
	// TODO: start thread to create octree
}

Graph::~Graph()
{
	delete(octree);
	for (Face *f : faceVector) {
		delete(f);
	}
}


void Graph::ConcatenateModelMeshes(const Model &model)
{
	std::vector<Mesh> meshes;
	for (Mesh m : model.meshes) {
		meshes.push_back(m);
	}

	int translateSize = 0, indicesSize = 0;
	for (Mesh mesh : meshes) {
		translateSize += mesh.vertices.size();
		indicesSize += mesh.indices.size();
	}

	std::vector<int> translate;
	translate = constructUniqueVertices(translateSize, meshes);
	constructUniqueFaces(indicesSize, translate, meshes);
}

std::vector<int> Graph::constructUniqueVertices(int size, std::vector<Mesh> &meshes)
{
	std::vector<int> translate;
	translate.reserve(size);
	std::unordered_multimap<float, int> vertexMap;
	int vIndex = 0;

	for (int i = 0; i < meshes.size(); i++) {
		Mesh *mesh = &meshes[i];
		for (Vertex v : mesh->vertices) {
			bool notFound = true;
			float key = v.Position.x;
			auto range = vertexMap.equal_range(key);

			//check vertexMap for key, find any equal vertices
			for (auto r = range.first; r != range.second; r++) {
				int index = r->second;
				// if found, set translate[tIndex] = value
				if (pointsEqual(v.Position, nodes[index]->vertex.Position)) {
					translate.push_back(index);
					notFound = false;
					break;
				}
			}
			//if not found, insert this point and set translate[tIndex] = vIndex
			if (notFound) {
				translate.push_back(vIndex);
				vertexMap.insert(std::pair<float, int>(key, vIndex));
				nodes.push_back(new Node(vIndex, v));
				vIndex++;
			}
		}
	}

	return translate;
}

void Graph::constructUniqueFaces(int size, std::vector<int> &translate, std::vector<Mesh> &meshes)
{
	std::vector<unsigned int> indices;
	indices.reserve(size);
	int offset = 0;
	for (int i = 0; i < meshes.size(); i++) {
		for (int j : meshes[i].indices) {
			indices.push_back(translate[j + offset]);
		}
		offset += meshes[i].vertices.size();
	}

	std::unordered_multimap<int, Face*> faceMap;
	for (int i = 0; i < indices.size(); i += 3) {
		bool notFound = true;
		int key = indices[i];
		auto range = faceMap.equal_range(key);

		//check faceMap for key, then find any equal faces
		for (auto r = range.first; r != range.second; r++) {
			Face *f = r->second;
			if ((indices[i + 1] == f->index2) && (indices[i + 2] == f->index3)) {
				notFound = false;
				break;
			}
		}

		//if not found, insert this face
		if (notFound) {
			Face *face = new Face(key, indices[i + 1], indices[i + 2], nodes);
			faceMap.insert(std::pair<int, Face*>(key, face));

			int indexOfLastFace = faceVector.size();
			faceVector.push_back(face);
			nodes[face->index1]->addFace(indexOfLastFace);
			nodes[face->index2]->addFace(indexOfLastFace);
			nodes[face->index3]->addFace(indexOfLastFace);
		}
	}
}

bool Graph::pointsEqual(glm::vec3 p, glm::vec3 q)
{
	//return !glm::all(glm::equal(q, p));
	bool a = false;
	bool b = false;
	bool c = false;
	if (std::abs(p.x - q.x) == 0.0) {
		a = true;
	}
	if (std::abs(p.y - q.y) == 0.0) {
		b = true;
	}
	if (std::abs(p.z - q.z) == 0.0) {
		c = true;
	}

	return (a && b && c);
}

void Graph::buildOctree()
{
	octree = new Octree(&nodes, &faceVector, getRange());
}

float Graph::getRange()
{
	if (Range <= 0.0) {
		float deltaX = 0.0, deltaY = 0.0, deltaZ = 0.0;
		for (Node *n : nodes) {
			if (n) {
				glm::vec3 vert = n->vertex.Position;

				int t = std::abs(vert.x);
				if (t > deltaX)
					deltaX = t;
				t = std::abs(vert.y);
				if (t > deltaY)
					deltaY = t;
				t = std::abs(vert.z);
				if (t > deltaZ)
					deltaZ = t;
			}
		}

		Range = 2.0 * std::max({ deltaX, deltaY, deltaZ });
	}
	return Range;
}

void Graph::populateConnections()
{
	std::vector<ordered_set> connections;
	connections.assign(nodes.size(), ordered_set());
	for (int i = 0; i < faceVector.size(); i += 3) {
		int a = faceVector[i]->index1;
		int b = faceVector[i]->index2;
		int c = faceVector[i]->index3;

		connections[a].insert(b);
		connections[a].insert(c);

		// this preserves winding order in the first two connections-
		// required to rebuild normals.
		connections[b].insert(c);
		connections[b].insert(a);

		connections[c].insert(b);
		connections[c].insert(a);
	}

	for (int n = 0; n < nodes.size(); n++) {
		for (int i : connections[n].vector) {
			if (n != i)
				nodes[n]->addConnection(i);
		}
	}
}

void Graph::recalculateNormalsFromFaces()
{
	int invalid = 0;
	// this generates invalid normals for some models, which are then not rendered.  Reason unknown.
	for (int n = 0; n < nodes.size(); n++) {
		std::vector<int> tvect;
		for (int face : nodes[n]->faces) {
			tvect.push_back(face);
		}

		if (tvect.size() > 0 && faceVector[tvect[0]]) {
			glm::vec3 tNormal = faceVector[tvect[0]]->normal;
			for (int i = 1; i < tvect.size(); i++) {
				tNormal = tNormal + glm::normalize(faceVector[tvect[i]]->normal);
			}
			glm::vec3 newNormal = glm::normalize(tNormal);

			nodes[n]->vertex.Normal = newNormal;
		}
		else {
			invalid++; //vertex has no faces.
		}
	}
}

void Graph::scale(float displacement)
{
	for (Node *n : nodes) {
		// TODO: scale()
		//call relocateVert() with offset, newPos, newNormal, and list of faces

		// float dist = 0;
		//for each associated face
		// find intersect of face plane and vertex normal
		// dist = std::max(dist, intersection);
		glm::vec3 pos = n->vertex.Position;
		glm::vec3 norm = n->vertex.Normal;
		n->vertex.Position = pos + (norm * displacement);
	}
}

int Graph::CombineNodes(int n1, int n2)
{
	//change to relocateVert
	Node *node = nodeFromAverage(nodes[n1], nodes[n2]);
	nodes.push_back(node);

	CombineConnections(n1, n2, node);
	CombineFaces(n1, n2, node);

	delete(nodes[n1]);
	nodes[n1] = NULL;
	delete(nodes[n2]);
	nodes[n2] = NULL;
	return node->ID;
}

void Graph::CombineConnections(int n1, int n2, Node *node)
{
	int index = node->ID;
	ordered_set connections;
	for (int i : nodes[n1]->connections) {
		if (nodes[i] && i != n2) {
			connections.insert(i);
		}
	}
	for (int i : nodes[n2]->connections) {
		if (nodes[i] && i != n1) {
			connections.insert(i);
		}
	}
	for (int i : connections.vector) {
		node->connections.push_back(i);

		// rebuild all connections from attached nodes
		Node *c = nodes[i];
		for (int j = 0; c->connections.size(); j++) {
			int cIndex = c->connections[j];
			if (cIndex == n1 || cIndex == n2) {
				c->connections[j] = index;
				break;
			}
		}
	}
}

void Graph::CombineFaces(int n1, int n2, Node *node)
{
	// delete faces that belong to both nodes
	int index = node->ID;
	for (int i : nodes[n1]->faces) {
		if (faceVector[i]) {
			node->faces.insert(i);
		}
	}
	for (int i : nodes[n2]->faces) {
		if (faceVector[i]) {
			if (node->faces.count(i) == 1) {
				node->faces.erase(i);
				delete(faceVector[i]);
				faceVector[i] = NULL;
			}
			else {
				node->faces.insert(i);
			}
		}
	}

	// update all other faces to new values
	for (int i : node->faces) {
		Face *face = faceVector[i];
		int v1 = face->index1;
		int v2 = face->index2;
		int v3 = face->index3;

		if (v1 == n1 || v1 == n2)
			face->index1 = index;
		if (v2 == n1 || v2 == n2)
			face->index2 = index;
		if (v3 == n1 || v3 == n2)
			face->index3 = index;

		faceVector[i]->update(nodes);
	}
}

Node* Graph::nodeFromAverage(Node* n1, Node* n2)
{
	glm::vec3 norm = glm::normalize(n1->vertex.Normal + n2->vertex.Normal);
	glm::vec3 pos = (n1->vertex.Position + n2->vertex.Position) * 0.5f;

	return new Node(nodes.size(), pos, norm, false);
}

Node* Graph::nodeFromIntercept(Node* n1, Node* n2)
{
	// TODO: create node based on intercepts with farthest face plane
	// (same as scale())
	glm::vec3 pos = (n1->vertex.Position + n2->vertex.Position);
	pos *= 0.5;
	glm::vec3 norm = glm::normalize(n1->vertex.Normal + n2->vertex.Normal);
	float distance;

	glm::intersectRayPlane(pos, norm, n1->vertex.Position, n1->vertex.Normal, distance);
	if (distance > 0.0) {
		pos += norm * distance;
	}

	return new Node(nodes.size(), pos, norm, false);
}

Graph* Graph::ReduceFootprint()
{
	std::vector<Node*> newNodes;
	std::vector<int> translate; // on the theory that this is faster than a hashtable
	translate.reserve(nodes.size());
	int index = 0;

	for (Node *n : nodes) {
		if (n) {
			newNodes.push_back(n);
			translate.push_back(index);
			index++;
		}
		else {
			translate.push_back(-1);
		}
	}

	std::vector<Face*> newFaces = cleanFaces(newNodes, translate);
	cleanConnections(newNodes, translate);
	return new Graph(newNodes, newFaces);
}

void Graph::cleanConnections(std::vector<Node*> &nodeList, std::vector<int> &translate)
{
	std::vector<ordered_set> connections;
	connections.assign(nodeList.size(), ordered_set());
	for (int n = 0; n < nodeList.size(); n++) {
		for (int c : nodeList[n]->connections) {
			if (nodes[c]) {
				connections[n].insert(translate[c]);
				connections[translate[c]].insert(n);
			}
		}
	}

	for (int n = 0; n < nodeList.size(); n++) {
		Node *node = new Node(n, nodeList[n]->vertex.Position, nodeList[n]->vertex.Normal, false);
		for (int f : nodeList[n]->faces) {
			node->faces.insert(f);
		}

		for (int i : connections[n].vector) {
			if (nodeList[i])
				node->addConnection(i);
		}

		nodeList[n] = node;
	}
}

std::vector<Face*> Graph::cleanFaces(std::vector<Node*> &nodeList, std::vector<int> &translate)
{
	std::vector<Face*> faceList;
	std::vector<int> faceTran;
	faceTran.reserve(faceList.size());

	int index = 0;
	for (Face *f : faceVector) {
		if (f) {
			faceTran.push_back(index);
			f->index1 = translate[f->index1];
			f->index2 = translate[f->index2];
			f->index3 = translate[f->index3];

			if (!(f->index1 == f->index2 || f->index1 == f->index3 || f->index2 == f->index3)) {
				faceList.push_back(f);
				index++;
			}
		}
		else {
			faceTran.push_back(-1);
		}
	}

	int invalid = 0;
	for (Node* node : nodeList) {
		// Push the faces
		std::unordered_set<int> newFaces;
		for (int f : node->faces) {
			int tran = faceTran[f];
			if (tran != -1)
				newFaces.insert(tran);
		}
		node->faces = std::unordered_set<int>();

		for (int f : newFaces) {
			node->faces.insert(f);
		}
	}

	return faceList;
}

bool Graph::verifyFacesFromConnections(int node)
{
	if (nodes[node]) {
		std::unordered_set<int> connectionsFromFaces;
		std::unordered_set<int> uniqueConnections;

		for (int face : nodes[node]->faces) {
			if (face != -1 && faceVector[face]) {
				int facecon = faceVector[face]->index1;
				if (nodes[facecon])
					connectionsFromFaces.insert(facecon);
				facecon = faceVector[face]->index2;
				if (nodes[facecon])
					connectionsFromFaces.insert(facecon);
				facecon = faceVector[face]->index3;
				if (nodes[facecon])
					connectionsFromFaces.insert(facecon);
			}
		}
		connectionsFromFaces.erase(nodes[node]->ID);

		for (int c : nodes[node]->connections) {
			if (nodes[c])
				uniqueConnections.insert(c);
		}

		for (int c : uniqueConnections) {
			if (connectionsFromFaces.count(c) == 1 || c == node) {
				connectionsFromFaces.erase(c);
			}
			else {
				std::vector<Node*> nodeConnections;
				std::vector<Face*> faceConnections;
				std::vector<Node*> relatedNodes;

				for (int nc : nodes[node]->connections) {
					nodeConnections.push_back(nodes[nc]);
					//relatedNodes.push_back(nodes[nc]);
				}
				for (int nf : nodes[node]->faces) {
					faceConnections.push_back(faceVector[nf]);
					relatedNodes.push_back(nodes[faceVector[nf]->index1]);
					relatedNodes.push_back(nodes[faceVector[nf]->index2]);
					relatedNodes.push_back(nodes[faceVector[nf]->index3]);
				}

				if (nodes[c])

					return false;
			}
		}

		return connectionsFromFaces.size() == 0;
	}

	return true;
}