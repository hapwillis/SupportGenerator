#include "Graph.h"


Graph::Graph(Model &model)
{
	Range = model.AABBsize();
	int index = 0;
	nodes.reserve(model.denseVertices.size());
	for (Vertex *vertex : model.denseVertices) {
		nodes.push_back(new Node(index, vertex->Position, vertex->Normal, true));
		index++;
	}

	faceVector.reserve(std::ceil(model.denseIndices.size() / 3.0));
	for (int i = 0; i < model.denseIndices.size(); i += 3) {
		faceVector.push_back(new Face(model.denseIndices[i], 
								model.denseIndices[i + 1], 
								model.denseIndices[i + 2], nodes));

		int findex = faceVector.size() - 1;
		Face *face = faceVector.back();
		nodes[face->index1]->addFace(findex);
		nodes[face->index2]->addFace(findex);
		nodes[face->index3]->addFace(findex);
	}

	// Populate every node with its faces and connections:
	populateConnections();
	ReduceFootprint(); 
	//float time = glfwGetTime();
	recalculateNormalsFromFaces(); //WARNING: recalculating normals breaks manifold.
	//std::cout << "Time to recalculate normals: " << glfwGetTime() - time << std::endl;

	if (updateThread.joinable())
		updateThread.join();
	octree = new Octree();
	updateThread = std::thread(&Octree::update, octree, &nodes, &faceVector, getRange());
}

Graph::Graph(std::vector<Node*> &newNodes, std::vector<Face*> &faces)
{
	nodes.reserve(newNodes.size());
	faceVector.reserve(faces.size());
	int index = 0;

	// This constructor is safe, because it copies all the nodes/faces, but slow.
	for (Node *n : newNodes) {
		if (n) {
			nodes.push_back(new Node(index, n->vertex.Position, n->vertex.Normal, true));
		} else {
			nodes.push_back(NULL);
		}
		index++;
	}
	for (Face *f : faces) {
		if (f) {
			Face *newFace = new Face(f->index1, f->index2, f->index3, newNodes);

			if (nodes[f->index1] && nodes[f->index2] && nodes[f->index3]) {
				int faceIndex = faceVector.size();
				faceVector.push_back(newFace);

				nodes[f->index1]->faces.insert(faceIndex);
				nodes[f->index2]->faces.insert(faceIndex);
				nodes[f->index3]->faces.insert(faceIndex);
			}
		}
	}

	populateConnections();
	//ReduceFootprint();
	recalculateNormalsFromFaces(); 
}

Graph::~Graph()
{
	if (updateThread.joinable())
		updateThread.join();

	delete(octree);

	for (Node *n : nodes)
		delete n;
	for (Face *f : faceVector)
		delete f;
}

Octree* Graph::getOctree()
{
	if (updateThread.joinable())
		updateThread.join();

	return octree;
}

float Graph::getRange() 
{
	if (Range <= 0.0) {
		float delta = 0.0;
		for (Node *node : nodes) {
			glm::vec3 v = node->vertex.Position;
			delta = std::max({ delta, std::abs(v.x), std::abs(v.y), std::abs(v.z) });
		}

		Range = 2.0 * delta;
	}

	return Range;
}

void Graph::populateConnections()
{
	std::vector<ordered_set> connections;
	connections.assign(nodes.size(), ordered_set());
	for (int i = 0; i < faceVector.size(); i++) { 
		int a = faceVector[i]->index1;
		int b = faceVector[i]->index2;
		int c = faceVector[i]->index3;

		if (!(a < 0 || b < 0 || c < 0)) {
			// this order preserves winding order in the first two connections-
			// required to rebuild normals from only connections.
			connections[a].insert(b);
			connections[a].insert(c);

			connections[b].insert(c);
			connections[b].insert(a);

			connections[c].insert(b);
			connections[c].insert(a);
		}
	}

	for (int n = 0; n < nodes.size(); n++) {
		for (int i : connections[n].vector) {
			if (n != i)
				nodes[n]->addConnection(i);
		}
	}
}

void Graph::relocateNodeFromFaceIntercepts(Node* node, Node* nodeFaces, float offset)
{
	// TODO: blows up for some vertices.
	glm::vec3 faceOrigin = nodeFaces->vertex.Position;
	glm::vec3 pos = node->vertex.Position;
	glm::vec3 norm = node->vertex.Normal;
	float maxDist = 0.0f;

	for (int face : nodeFaces->faces) {
		if (faceVector[face]) {
			glm::vec3 faceNorm = faceVector[face]->normal;
			glm::vec3 offsetFaceOrigin = faceOrigin + offset * faceNorm;
			float distance = faceExclusionDist(pos, norm, offsetFaceOrigin, faceNorm);

			if (distance > maxDist)
				maxDist = distance;
		}
	}

	node->vertex.Position += norm * maxDist;
}

void Graph::recalculateNormalsFromFaces()
{
	for (Face *face : faceVector) {
		face->update(nodes);
	}

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
	}
}

void Graph::scale(float offset)
{
	recalculateNormalsFromFaces();
	// TODO: scale from offset to the original model... faster than fancy node relocations
	for (Node *n : nodes) {
		n->vertex.Position += offset * n->vertex.Normal;
		//relocateNodeFromFaceIntercepts(n, n, offset);
	}

	for (Face* f : faceVector) {
		f->update(nodes);
	}

	// TODO: get the scale earlier, or adjust octree so we don't have to do this.
	if (updateThread.joinable())
		updateThread.join();
	delete octree;
	octree = new Octree();
	octree->update(&nodes, &faceVector, getRange());
	//updateThread = std::thread(&Octree::update, octree, &nodes, &faceVector, getRange());
}

int Graph::CombineNodes(int n1, int n2)
{
	Node *node1 = nodes[n1];
	Node *node2 = nodes[n2];
	Node *node = nodeFromAverage(node1, node2);
	//relocateNodeFromFaceIntercepts(node, node1, 0.0f);
	//relocateNodeFromFaceIntercepts(node, node2, 0.0f);

	CombineConnections(n1, n2, node); // 344 milliseconds
	CombineFaces(n1, n2, node); // 363 milliseconds

	glm::vec3 tNormal(0.0f, 0.0f, 0.0f);
	for (int f : node->faces) {
		tNormal += faceVector[f]->normal;
	}
	node->vertex.Normal = glm::normalize(tNormal);

	delete(node1);
	nodes[n1] = NULL;
	delete(node2);
	nodes[n2] = NULL;
	return node->ID;
}

void Graph::CombineConnections(int n1, int n2, Node *node)
{
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

	int index = node->ID;
	for (int i : connections.vector) {
		node->connections.push_back(i);

		// rebuild all connections from attached nodes
		Node *c = nodes[i];
		bool found = false;
		for (int j = c->connections.size() - 1; j >= 0; j--) {
			int cIndex = c->connections[j];
			if (cIndex == n1 || cIndex == n2) {
				if (found) {
					c->connections.erase(c->connections.begin() + j);
				} else {
					c->connections[j] = index;
				}
			}
		}
	}
}

void Graph::CombineFaces(int n1, int n2, Node *node)
{
	// delete faces that belong to both nodes
	int index = node->ID;
	for (int i : nodes[n1]->faces) {
		if (i > 0 && faceVector[i]) {
			node->faces.insert(i);
		}
	}
	for (int i : nodes[n2]->faces) {
		if (i > 0 && faceVector[i]) {
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
		if (face) {
			int v1 = face->index1;
			int v2 = face->index2;
			int v3 = face->index3;

			if (nodes[v1] && nodes[v2] && nodes[v3]) {
				if (v1 == n1 || v1 == n2)
					face->index1 = index;
				if (v2 == n1 || v2 == n2)
					face->index2 = index;
				if (v3 == n1 || v3 == n2)
					face->index3 = index;

				faceVector[i]->update(nodes);
			}
			else {
				delete(face);
				faceVector[i] = NULL;
			}
		}
	}
}

Node* Graph::nodeFromAverage(Node* n1, Node* n2)
{
	glm::vec3 norm = glm::normalize(n1->vertex.Normal + n2->vertex.Normal);
	glm::vec3 pos = (n1->vertex.Position + n2->vertex.Position) * 0.5f;

	Node *node = new Node(nodes.size(), pos, norm, true);
	nodes.push_back(node);
	return node;
}

Node* Graph::nodeFromIntercept(Node* n1, Node* n2)
{
	// Create node based on intercepts with farthest face plane
	glm::vec3 pos = (n1->vertex.Position + n2->vertex.Position) * 0.5f;
	glm::vec3 norm = glm::normalize(n1->vertex.Normal + n2->vertex.Normal);
	
	float maxDist = 0.0f;
	for (int face : n1->faces) {
		float distance = faceExclusionDist(pos, norm, n1->vertex.Position, faceVector[face]->normal);
		//glm::intersectRayPlane(pos, norm, n1->vertex.Position, faceVector[face]->normal, distance);
		if (distance > maxDist)
			maxDist = distance;
	}
	for (int face : n2->faces) {
		float distance = faceExclusionDist(pos, norm, n1->vertex.Position, faceVector[face]->normal);
		//glm::intersectRayPlane(pos, norm, n2->vertex.Position, faceVector[face]->normal, distance);
		if (distance > maxDist)
			maxDist = distance;
	}

	Node *node = new Node(nodes.size(), pos + norm * maxDist, norm, true);
	nodes.push_back(node);
	return node;
}

float Graph::faceExclusionDist(glm::vec3 pointPosition, glm::vec3 pointNormal, glm::vec3 faceVertex, glm::vec3 faceNormal)
{
	// NB: undefined behavior if pointNormal is not normalized.
	float f = glm::dot(faceVertex - pointPosition, faceNormal);
	if (f < 0.00000001f) // nonintersection or negative
		return 0.0f;

	float a = glm::dot(pointNormal, faceNormal);
	if (a < 0.00000001f) // normal parallel to face
		return 0.0f;

	float distance = f / a; 

	if (distance > 4.0f)
		std::cout << "ffffff" << std::endl;

	return distance;
}

void Graph::ReduceFootprint()
{
	std::vector<int> translate; // faster, but larger than a hashtable.
	translate.reserve(nodes.size());

	int index = 0;
	for (Node *n : nodes) {
		if (n) {
			n->ID = index;
			nodes[index] = n;
			translate.push_back(index);
			index++;
		} else {
			translate.push_back(-1);
		}
	}
	nodes.resize(index);
	nodes.shrink_to_fit();

	cleanFaces(translate);
	cleanConnections(translate);
	faceVector.shrink_to_fit();

	if (updateThread.joinable())
		updateThread.join();
	octree = new Octree();
	updateThread = std::thread(&Octree::update, octree, &nodes, &faceVector, getRange());
}

void Graph::cleanConnections(std::vector<int> &translate)
{
	std::vector<ordered_set> connections;
	connections.assign(nodes.size(), ordered_set());
	for (int n = 0; n < nodes.size(); n++) {
		for (int c : nodes[n]->connections) {
			if (nodes[translate[c]]) {
				connections[n].insert(translate[c]);
				connections[translate[c]].insert(n);
			}
		}
	}

	for (Node *node : nodes) {
		node->connections.clear();
		for (int i : connections[node->ID].vector) {
			if (nodes[i])
				node->connections.push_back(i);
		}
	}
}

void Graph::cleanFaces(std::vector<int> &translate)
{
	std::vector<Face*> faceList;
	std::vector<int> faceTran;
	faceTran.reserve(faceList.size());

	int index = 0;
	for (Face *f : faceVector) {
		if (f) {
			f->index1 = translate[f->index1];
			f->index2 = translate[f->index2];
			f->index3 = translate[f->index3];

			if (!(	f->index1 == f->index2 || 
					f->index1 == f->index3 || 
					f->index2 == f->index3 )) {

				faceTran.push_back(index);
				faceList.push_back(f);
				index++;
			} else {
				faceTran.push_back(-1);
			}
		} else {
			faceTran.push_back(-1);
		}
	}
	faceVector = faceList;

	for (Node* node : nodes) {
		// Push the faces
		std::unordered_set<int> newFaces;
		for (int f : node->faces) {
				int tran = faceTran[f];
				if (tran > -1)
					newFaces.insert(tran);
		}
		node->faces.clear();

		for (int f : newFaces) {
			node->faces.insert(f);
		}
	}
}

bool Graph::test()
{
	// TODO: test function
	testPopulateConnections();

	for (Node *node : nodes) {
		for (int c : node->connections) {
			if (nodes[c]) {
				int thisNodeID = node->ID;
				bool found = false;
				for (int n : nodes[c]->connections) {
					if (n == thisNodeID)
						found = true;
				}
				if (!found)
					std::cout << "Connection is only one way!" << std::endl;
			} else {
				std::cout << "Connection to node which DNE!" << std::endl;
			}
		}
	}
	return false;
}

bool Graph::testCleanFaces()
{
	// TODO: test cleanfaces
	return false;
}

bool Graph::testCleanConnections()
{
	// TODO: test clean connections
	return false;
}

bool Graph::testPopulateConnections()
{
	int failedP = 0;
	for (Node *node : nodes) {
		bool fail = false;
		int comb = node->ID;
		std::unordered_set<int> faceConnections;
		for (int faceIndex : node->faces) {
			if (faceIndex < 0) { //invalid face index/temp face
				failedP++;
				fail = true;
				break;
			}
			else {
				if (!faceVector[faceIndex]) { //face does not exist
					failedP++;
					fail = true;
					break;
				}
				// face is not attached to this node
				if (!(faceVector[faceIndex]->index1 == comb || faceVector[faceIndex]->index2 == comb || faceVector[faceIndex]->index3 == comb)) {
					failedP++;  
					fail = true;
					break;
				}
				// face is a line or point
				if (faceVector[faceIndex]->index1 == faceVector[faceIndex]->index2 ||
					faceVector[faceIndex]->index1 == faceVector[faceIndex]->index3 ||
					faceVector[faceIndex]->index2 == faceVector[faceIndex]->index3) {

					fail = true;
					failedP++;
					break;
				}

				faceConnections.insert(faceVector[faceIndex]->index1);
				faceConnections.insert(faceVector[faceIndex]->index2);
				faceConnections.insert(faceVector[faceIndex]->index3);
			}
		}
		// faceConnections should be the same as connections except for node->ID
		if (!fail && faceConnections.size() - 1 != node->connections.size())
			failedP++; 
	}

	std::cout << failedP << " out of " << nodes.size() << " Nodes were improperly populated." << std::endl;
	return failedP < 1;
}

bool Graph::testCombineNodes()
{
	// TODO: test combinenodes
	return false;
}

bool Graph::validateNodeCombination(Node * n1, Node * n2, Node * combined)
{
	int comb = combined->ID;
	if (n1->ID == comb || n1->ID == comb)
		return false;

	if (comb < 0 || comb > nodes.size())
		return false;

	if (glm::length(combined->vertex.Normal) > 1.0000001f)
		return false; 

	if (n1->faces.size() + n2->faces.size() - 4 != combined->faces.size())
		return false; 

	for (int faceIndex : combined->faces) {
		if (faceIndex < 0 || faceIndex > faceVector.size())
			return false;

		if (!faceVector[faceIndex])
			return false; 

		if (n1->faces.count(faceIndex) < 1 && n2->faces.count(faceIndex) < 1)
			return false;

		if (!(faceVector[faceIndex]->index1 == comb || faceVector[faceIndex]->index2 == comb || faceVector[faceIndex]->index3 == comb))
			return false; 
	}

	if (n1->connections.size() + n2->connections.size() - 2 != combined->connections.size())
		return false; 

	std::unordered_set<int> conSet;
	for (int conn : combined->connections) {
		if (conn < 0 || conn > nodes.size())
			return false;

		if (!nodes[conn])
			return false;

		if (conSet.count(conn) == 1 && !(conn == n1->ID || conn == n2->ID))
			return false;

		conSet.insert(conn);
	}

	return true;
}

bool Graph::validateNode(Node * node)
{
	int comb = node->ID;
	std::unordered_set<int> faceConnections;

	for (int faceIndex : node->faces) {
		if (faceIndex < 0)
			return false;
		if (!faceVector[faceIndex])
			return false;
		if (!(faceVector[faceIndex]->index1 == comb || faceVector[faceIndex]->index2 == comb || faceVector[faceIndex]->index3 == comb))
			return false;
		if (faceVector[faceIndex]->index1 == faceVector[faceIndex]->index2 ||
			faceVector[faceIndex]->index1 == faceVector[faceIndex]->index3 || 
			faceVector[faceIndex]->index2 == faceVector[faceIndex]->index3) {

			return false;
		}

		faceConnections.insert(faceVector[faceIndex]->index1);
		faceConnections.insert(faceVector[faceIndex]->index2);
		faceConnections.insert(faceVector[faceIndex]->index3);
	}

	// faceConnections should be the same as connections except for node->ID
	if (faceConnections.size() - 1 != node->connections.size())
		return false;

	return true;
}

bool Graph::testReduceFootprint()
{
	// TODO: test reducefootprint
	return false;
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