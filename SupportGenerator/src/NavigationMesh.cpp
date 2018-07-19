#include "NavigationMesh.h"


Edge::Edge(weak_ptr<Node> a, weak_ptr<Node> b, float length) : a_(a), b_(b), length_(length)
{

}


EdgeContainer::EdgeContainer(weak_ptr<Node> a, weak_ptr<Node> b, glm::vec3 e1, glm::vec3 e2) :
	a_(a), b_(b)
{
	if (e1.x < e2.x) {
		negVert_ = e1;
		posVert_ = e2;
	}
	else if (e1.x == e2.x) {
		if (e1.y < e2.y) {
			negVert_ = e1;
			posVert_ = e2;
		}
		else if (e1.y == e2.y) {
			if (e1.z < e2.z) {
				negVert_ = e1;
				posVert_ = e2;
			}
			else {
				negVert_ = e2;
				posVert_ = e1;
			}
		}
		else {
			negVert_ = e2;
			posVert_ = e1;
		}
	}
	else {
		negVert_ = e2;
		posVert_ = e1;
	}

	length_ = glm::distance(e1, e2);
}


EdgeSet::~EdgeSet()
{
	for (EdgeContainer *e : edges_) {
		delete(e);
	}
}

void EdgeSet::insert(EdgeContainer *e1)
{
	bool notFound = true;
	float key = e1->posVert_.x; // TODO: this really isn't a great key.
	auto range = edgeMap_.equal_range(key);

	//check edgeMap for key, then find any equal edges
	for (auto r = range.first; r != range.second; r++) {
		EdgeContainer *e2 = r->second;

		bool posEqual = glm::all(glm::equal(e1->posVert_, e2->posVert_));
		bool negEqual = glm::all(glm::equal(e1->negVert_, e2->negVert_));
		if (posEqual && negEqual) {
			notFound = false;
			break;
		}
	}

	//if not found, insert this point and set translate[tIndex] = vIndex
	if (notFound) {
		edgeMap_.insert(std::pair<float, EdgeContainer*>(key, e1));
		edges_.push_back(e1);
	}
}

// "Hash" function for EdgeContainers.
// Avoid if possible- inefficient.
size_t KeyFuncs::operator()(const EdgeContainer& e)const
{
	std::string s("");
	glm::vec3 n = e.negVert_;
	glm::vec3 p = e.posVert_;
	s.append(std::to_string(n.x));
	s.append(std::to_string(n.y));
	s.append(std::to_string(n.z));
	s.append(std::to_string(p.x));
	s.append(std::to_string(p.y));
	s.append(std::to_string(p.z));

	return std::hash<std::string>()(s);
}

// Custom comparator for sets to prevent equivalength_t edges.
bool KeyFuncs::operator()(const EdgeContainer& a, const EdgeContainer& b)const
{
	bool v1 = glm::all(glm::equal(a.negVert_, b.negVert_));
	bool v2 = glm::all(glm::equal(a.posVert_, b.posVert_));
	return v1 && v2;
}

// Custom comparator for std::priority_heap for EdgeContainers.
bool operator<(const EdgeContainer& a, const EdgeContainer& b)
{
	return a.length_ > b.length_;
}

// Custom comparator for std::priority_heap for Edges.
bool operator<(const Edge& a, const Edge& b)
{
	return a.length_ > b.length_;
}


NavigationMesh::NavigationMesh(vector<shared_ptr<Vertex>> &vertices, vector<int> &indices) {
	//double time = glfwGetTime();
	// 27.6864 seconds to build graph
	graph_ = shared_ptr<Graph>(new Graph(vertices, indices));
	//std::cout << "Time to build Graph: " << glfwGetTime() - time << std::endl;
}


NavigationMesh::~NavigationMesh()
{
	
}

// Returns a new simplified (decimated), scaled and cleaned Graph.
shared_ptr<Graph> NavigationMesh::getSimpleGraph(float minlength_)
{
	//float time = glfwGetTime();
	// 57.52 seconds to run decimateMesh

  shared_ptr<Graph> graph = shared_ptr<Graph>(new Graph(graph_->nodes_));
	decimateMesh(graph, minlength_);
	//std::cout << "Time to reduce mesh: " << glfwGetTime() - time << std::endl;

	return graph;
}

// Simplifies a mesh by removing the smallest edge until all edges are longer than minlength_.
void NavigationMesh::decimateMesh(shared_ptr<Graph> graph, float minlength_)
{
	EdgeHeap edgeHeap(graph->nodes_, graph->faces_); // 184 milliseconds
	float smallestEdge = 0.0f;
	while (!edgeHeap.empty() && smallestEdge < minlength_) {
		//	pop() until you get a valid edge (ie both vertices exist)
		Edge e = edgeHeap.pop();
		while (!edgeValid(e, graph) && !edgeHeap.empty()) { // 736 milliseconds
			e = edgeHeap.pop();
		}

    shared_ptr<Node> newNode = graph->CombineNodes(e.a_.lock(), e.b_.lock()); // 847 milliseconds
		glm::vec3 a = newNode->vertex_.Position;

		//	add all the new edges to the minheap
		for (weak_ptr<Node> connection : newNode->connections_) {
      auto node = connection.lock();
			if (node) {
				glm::vec3 b = node->vertex_.Position;
				float length_ = glm::distance(a, b);
		
				edgeHeap.push(Edge(weak_ptr<Node>(newNode), connection, length_));
			}
		}

		smallestEdge = e.length_;
	} 

	reduceFootprint(graph);
}

// Removes nodes from memory
void NavigationMesh::reduceFootprint(shared_ptr<Graph> graph) {
  int index = 0;
  for (shared_ptr<Node> n : graph->nodes_) {
    if (n && !n->removed) {
      graph->nodes_[index] = n;
      index++;
    }
  }

  graph->nodes_.resize(index);
  graph->nodes_.shrink_to_fit();
}

// Creates a renderable mesh from a Graph, and scales it by offset.
shared_ptr<Mesh> NavigationMesh::convertToMesh(shared_ptr<Graph> graph, float offset)
{
	//float time = glfwGetTime();
	graph->recalculateNormalsFromFaces();
	graph->scale(offset);
	std::vector<Vertex> vertices;
	vertices.reserve(graph->nodes_.size());

	for (shared_ptr<Node> node : graph->nodes_) {
		vertices.push_back(Vertex(node->vertex_.Position, node->vertex_.Normal, true));
	}

	std::vector<unsigned int> indices;
	facesToIndices(graph, indices);
	mesh_ = shared_ptr<Mesh>(new Mesh(vertices, indices));
	//std::cout << "Time to convert mesh: " << glfwGetTime() - time << std::endl;
	return mesh_;
}

// Removes Nodes which are below the print bed.
void NavigationMesh::PruneSubBedVertices(glm::mat4 model)
{

	// TODO: pruning
}

// Adds indices to a given vector, generated from the faces of a Graph g.
void NavigationMesh::facesToIndices(shared_ptr<Graph> graph, std::vector<unsigned int> &indices)
{
	for (shared_ptr<Face> face : graph->faces_) {
		if (face && graph->nodes_[face->index1] && 
					graph->nodes_[face->index2] && 
					graph->nodes_[face->index3]) {

			indices.push_back(face->index1);
			indices.push_back(face->index2);
			indices.push_back(face->index3);
		}
	}
}

// Returns true if the nodes composing an edge exist.  
bool NavigationMesh::edgeValid(Edge edge, shared_ptr<Graph> graph)
{
	return edge.a_.lock() && edge.b_.lock();
}

// Compares a std::priority_queue to my implementation.  
// Utility class to ensure they both still work while optimizing.
// TODO: move this somewhere else (eg under a ctest)
void NavigationMesh::heapTest()
{
	EdgeHeap customClass;

	if (customClass.heapTest()) {
		std::cout << "Custom edgeheap works correctly." << std::endl;
	} else {
		std::cout << "Error! Custom edgeheap failed!" << std::endl;
	}

	std::priority_queue<Edge> builtInClass;
	std::default_random_engine gen;
	std::uniform_real_distribution<double> rand(0.0f, 1000.0f);
	std::vector<Edge> testEdges;
	int testNumber = 10000;
	testEdges.reserve(testNumber * 1.5);
	for (int i = 0; i < testNumber; i++) {
		testEdges.push_back(Edge(weak_ptr<Node>(), weak_ptr<Node>(), rand(gen)));
	}

	if (!builtInClass.empty()) 
		std::cout << "Error!  Edgeheap not empty!" << std::endl;
	
	for (Edge e : testEdges) {
		builtInClass.push(e);
	}
	if (builtInClass.empty()) {
		std::cout << "Error!  Edgeheap empty!" << std::endl;
	}

	int emptyHeapErrors = 0;
	int outOfOrderErrors = 0;
	float heapMin = builtInClass.top().length_;
	builtInClass.pop();
	for (int i = 0; i < testNumber - 1; i++) {
		if (builtInClass.empty())
			emptyHeapErrors++;
		if (builtInClass.top().length_ < heapMin)
			outOfOrderErrors++;
		
		heapMin = builtInClass.top().length_;
		builtInClass.pop();
	}
	if (emptyHeapErrors > 0)
		std::cout<< emptyHeapErrors << " Edgeheap empty errors!" << std::endl;
	if (outOfOrderErrors > 0)
		std::cout << outOfOrderErrors << " edges returned out of order!" << std::endl;
	if (!builtInClass.empty()) {
		std::cout << "Error!  Edgeheap not empty!" << std::endl;
	}

	int size = 0;
	for (Edge e : testEdges) {
		size++;
		builtInClass.push(e);
	}
	for (int i = 0; i < testNumber / 2; i++) {
		size--;
		builtInClass.pop();
	}
	for (int i = 0; i < testNumber / 2; i++) {
		size++;
		testEdges.push_back(Edge(weak_ptr<Node>(), weak_ptr<Node>(), rand(gen)));
		builtInClass.push(testEdges.back());
	}
	size--;
	heapMin = builtInClass.top().length_;
	builtInClass.pop();
	outOfOrderErrors = 0;
	while (!builtInClass.empty()) {
		size--;
		if (builtInClass.top().length_ < heapMin)
			outOfOrderErrors++;
		builtInClass.pop();
	}
	if (outOfOrderErrors > 0)
		std::cout << outOfOrderErrors << " edges returned out of order!" << std::endl;
	if (size != 0) {
		std::cout << "Error!  Edges pushed != Edges popped!" << std::endl;
	}
}

// Draws the navigation surface in blue, as a wireframe.
void NavigationMesh::Draw(DefaultShader shader)
{
	if (mesh_) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		shader.setVec4("color", glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
		mesh_->Draw(shader);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
}


EdgeHeap::EdgeHeap()
{
	heap_.reserve(defaultSize);
	heap_.push_back(Edge(weak_ptr<Node>(), weak_ptr<Node>(), 0.0f)); //fill up spot zero
}


EdgeHeap::EdgeHeap(vector<shared_ptr<Node>> &nodes, vector<shared_ptr<Face>> &faces)
{
	heap_.reserve(2 * faces.size() + 2);
	heap_.push_back(Edge(weak_ptr<Node>(), weak_ptr<Node>(), 0.0f)); //fill up spot zero
	EdgeSet edges;

	for (unsigned int i = 0; i < faces.size(); i += 3) {
    shared_ptr<Face> face = faces[i];
		glm::vec3 a = face->vertex1->Position;
		glm::vec3 b = face->vertex2->Position;
		glm::vec3 c = face->vertex3->Position;

    weak_ptr<Node> n1 = nodes[face->index1];
    weak_ptr<Node> n2 = nodes[face->index2];
    weak_ptr<Node> n3 = nodes[face->index3];
		
		if (face->index1 != face->index2)
			edges.insert(new EdgeContainer(n1, n2, a, b));
		if (face->index2 != face->index3)
			edges.insert(new EdgeContainer(n2, n3, b, c));
		if (face->index3 != face->index1)
			edges.insert(new EdgeContainer(n3, n1, c, a));
	}

	// Insert the unique edges to a heap
	for (EdgeContainer *e : edges.edges_) {
		push(Edge(e->a_, e->b_, e->length_));
	}
}


EdgeHeap::~EdgeHeap()
{
}


bool EdgeHeap::empty()
{
	return heap_.size() == 1;
}


Edge EdgeHeap::pop() // 733 milliseconds
{
	// This method is O(2logn).  
	// An O(logn) method is to promote children until you reach a leaf,
	// then filling that leaf with another leaf and bubbling it up.
	Edge topEdge = heap_[1];
	Edge bubbleEdge = heap_.back();
	float bubblelength_ = bubbleEdge.length_;
	heap_[1] = bubbleEdge;
	heap_.pop_back();

	int index = 1;
	int leftChild = 2;
	int minIndex = index;

	while (leftChild < heap_.size()) {
		float minValue = bubblelength_;

		float childValue = heap_[leftChild].length_; // 164 milliseconds
		if (childValue < minValue) { // 410 milliseconds
			minValue = childValue;
			minIndex = leftChild;
		}
		
		int rightChild = leftChild + 1;
		if (rightChild < heap_.size()) {
			childValue = heap_[rightChild].length_;
			if (childValue < minValue) {
				minIndex = rightChild;
			}
		}

		if (minIndex == index)
			break;

		heap_[index] = heap_[minIndex];
		index = minIndex;
		leftChild = index << 1;
	}

	heap_[minIndex] = bubbleEdge;

	return topEdge;
}


void EdgeHeap::push(Edge e)
{
	// TODO: investigate deleting invalid edges as they are discovered-
  // that might be faster than calling pop each time.  It's tricky, though.
  int index = heap_.size();
	heap_.push_back(e);
	int parent = index >> 1; // Divide by two

	if (parent > 0) {
		// Can this be threaded?  It's overkill, but if it lets
		// the caller start faster then it might be worth it.
		// You'd just join the thread at the top of pop().
		Edge swapHolder = heap_[parent];
		float insertValue = e.length_;
		float parentValue = heap_[parent].length_;

		while (insertValue < parentValue) { // i/2 is equivalength_t to floor(i/2)
			heap_[index] = swapHolder;
			heap_[parent] = e;

			index = parent;
			// If index is at the top of the heap (ie at 1):
			// Let parentValue = insertValue so the loop breaks
			// Alternately you could branch and break, but that's more inside the loop
			if (index > 1) 
				parent >>= 1;

			swapHolder = heap_[parent];
			parentValue = swapHolder.length_;
		}
	}
}

// Tests edgeHeap for functionality
bool EdgeHeap::heapTest()
{
	std::default_random_engine gen;
	std::uniform_real_distribution<double> rand(0.0, 1000.0);
	std::vector<Edge> testEdges;
	int testNumber = 10000;

	testEdges.reserve(testNumber * 1.5);
	for (int i = 0; i < testNumber; i++) {
		testEdges.push_back(Edge(weak_ptr<Node>(), weak_ptr<Node>(), rand(gen)));
	}

	if (!empty()) {
		std::cout << "Error!  Edgeheap not empty!" << std::endl;
		return false;
	}

	for (Edge e : testEdges) {
		push(e);
	}
	if (empty()) {
		std::cout << "Error!  Edgeheap empty!" << std::endl;
		return false;
	}

	float heapMin = pop().length_;
	for (int i = 0; i < testNumber - 1; i++) {
		if (empty()) {
			std::cout << "Error!  Edgeheap empty!" << std::endl;
			return false;
		}
			
		Edge e = pop();
		if (e.length_ < heapMin) {
			std::cout << "Error!  Edges returned out of order!" << std::endl;
			return false;
		}

		heapMin = e.length_;
	}
	if (!empty()) {
		std::cout << "Error!  Edgeheap not empty!" << std::endl;
		return false;
	}
		
	int size = 0;
	for (Edge e : testEdges) {
		size++;
		push(e);
	}
	for (int i = 0; i < testNumber / 2; i++) {
		size--;
		pop();
	}
	for (int i = 0; i < testNumber / 2; i++) {
		size++;
		testEdges.push_back(Edge(weak_ptr<Node>(), weak_ptr<Node>(), rand(gen)));
		push(testEdges.back());
	}
	size--;
	heapMin = pop().length_;
	while (!empty()) {
		size--;
		Edge e = pop();
		if (e.length_ < heapMin) {
			std::cout << "Error!  Edges returned out of order!" << std::endl;
			return false;
		}
	}
	if (size != 0) {
		std::cout << "Error!  Edges pushed != Edges popped!" << std::endl;
		return false;
	}

	return true;
}