#include "Graph.h"


Node::Node(int index, Vertex v) :
  vertex_(v) {
}

Node::Node(int index, glm::vec3 pos, glm::vec3 norm, bool wireframe) {
  vertex_ = Vertex(pos, norm, wireframe);
}

void Node::addConnection(shared_ptr<Node> node) {
  connections_.insert(node);
}

void Node::addConnection(weak_ptr<Node> node) {
  connections_.insert(node);
}


Graph::Graph(vector<shared_ptr<Vertex>> &vertices, vector<int> &indices) {
	int index = 0;
	nodes_.reserve(vertices.size());
	for (shared_ptr<Vertex> vertex : vertices) {
		nodes_.emplace_back(new Node(index, vertex->Position, vertex->Normal, true));
		index++;
	}

	faces_.reserve(std::ceil(indices.size() / 3.0));
	for (int i = 0; i < indices.size(); i += 3) {
		faces_.emplace_back(new Face(indices[i], 
								indices[i + 1], 
								indices[i + 2], vertices));
	}

	// Populate every node with its faces and connections:
  for (shared_ptr<Face> face : faces_) {
    face->update(vertices);
  }
	populateConnections();
	recalculateNormalsFromFaces(); 
}

Graph::Graph(vector<shared_ptr<Node>> nodes) {
	nodes_.reserve(nodes.size());
	int index = 0;

	// This constructor is safe, because it copies all the nodes/faces, but slow.
	for (shared_ptr<Node> n : nodes) {
    nodes_.emplace_back(new Node(index, n->vertex_.Position, n->vertex_.Normal, true));
    index++;
	}

	populateConnections();
	recalculateNormalsFromFaces(); 
}


Graph::~Graph() {
}

// Returns the size of a doubly-large axis aligned bounding cube
float Graph::getRange() {
	if (range_ <= 0.0) {
		float delta = 0.0;
		for (shared_ptr<Node> node : nodes_) {
			glm::vec3 v = node->vertex_.Position;
			delta = std::max({ delta, std::abs(v.x), std::abs(v.y), std::abs(v.z) });
		}

		range_ = 2.0 * delta;
	}

	return range_;
}

// For each Node, populates connections from a vector of faces.
void Graph::populateConnections() {
	std::vector<ordered_set> connections;
	connections.assign(nodes_.size(), ordered_set());
	for (int i = 0; i < faces_.size(); i++) { 
		int a = faces_[i]->index1;
		int b = faces_[i]->index2;
		int c = faces_[i]->index3;

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

	for (int n = 0; n < nodes_.size(); n++) {
		for (int i : connections[n].vector) {
			if (n != i)
				nodes_[n]->addConnection(nodes_[i]);
		}
	}
}

// Scales by displacing vertices along their normal.  
// NB that this is not a standard cartesian scaling.
// Also rebuilds the octree.
void Graph::scale(float offset) {
	recalculateNormalsFromFaces();
	// TODO: scale from offset to the original model... faster than fancy node relocations
	for (shared_ptr<Node> n : nodes_) {
		n->vertex_.Position += offset * n->vertex_.Normal;
		//relocateNodeFromFaceIntercepts(n, n, offset);
	}
}

// For each Node, recalculates the normal from the normals of each attached face.
void Graph::recalculateNormalsFromFaces() {
  vector<vector<shared_ptr<Face>>> faceVector;
  faceVector.reserve(nodes_.size());
  faceVector.assign(nodes_.size(), vector<shared_ptr<Face>>());

  for (shared_ptr<Face> f : faces_) {
    if (f && nodes_[f->index1] && nodes_[f->index2] && nodes_[f->index3]) {
      faceVector[f->index1].push_back(f);
      faceVector[f->index2].push_back(f);
      faceVector[f->index3].push_back(f);
    }
  }

  for (int n = 0; n < faceVector.size(); n++) {
    auto faces = faceVector[n];
    vec3 vertexNormal(0.0f, 0.0f, 0.0f);
    
    for (shared_ptr<Face> face : faces) {
      vertexNormal += glm::normalize(face->normal);
    }

    nodes_[n]->vertex_.Normal = glm::normalize(vertexNormal);
  }
}

// Removes an edge by combining two nodes.
shared_ptr<Node> Graph::CombineNodes(shared_ptr<Node> n1, shared_ptr<Node> n2) {
  shared_ptr<Node> node = nodeFromAverage(n1, n2);

  for (weak_ptr<Node> c : n1->connections_) {
    if (c.lock() && c.lock() != n2) {
      node->addConnection(c);
    }
  }

  for (weak_ptr<Node> c : n2->connections_) {
    if (c.lock() && c.lock() != n1) {
      node->addConnection(c);
    }
  }

  n1->removed = true;
  n2->removed = true;

	return node;
}

// Returns a new node that is offset to the average position of the given nodes.
shared_ptr<Node> Graph::nodeFromAverage(shared_ptr<Node> n1, shared_ptr<Node> n2) {
	glm::vec3 norm = glm::normalize(n1->vertex_.Normal + n2->vertex_.Normal);
	glm::vec3 pos = (n1->vertex_.Position + n2->vertex_.Position) * 0.5f;

  shared_ptr<Node> node = shared_ptr<Node>(new Node(nodes_.size(), pos, norm, true));
  node->parentA = n1;
  node->parentB = n2;
	nodes_.push_back(node);

	return node;
}


