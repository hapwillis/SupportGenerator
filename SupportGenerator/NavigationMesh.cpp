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

NavigationMesh::NavigationMesh(Model &newModel)
{
	heapTest();
	model = &newModel; 
	//double time = glfwGetTime();
	// 27.6864 seconds to build graph
	graph = new Graph(*model);
	//std::cout << "Time to build Graph: " << glfwGetTime() - time << std::endl;
}

NavigationMesh::~NavigationMesh()
{
	
}

Graph * NavigationMesh::getSimpleGraph(float minLength)
{
	//float time = glfwGetTime();
	// 57.52 seconds to run decimateMesh
	Graph *navGraph = decimateMesh(minLength);
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

Graph* NavigationMesh::decimateMesh(float minLength)
{
	// TODO: rewrite this so it doesn't corrupt the graph nodes.
	std::priority_queue<Edge> edgeHeap;
	// TODO: heap appears to be broken
	initializeHeap(edgeHeap); // TODO: write custom heap that uses pointers

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

void NavigationMesh::PruneSubBedVertices(glm::mat4 model)
{

	// TODO: pruning
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
		testEdges.push_back(Edge(0, 0, rand(gen)));
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
	float heapMin = builtInClass.top().length;
	builtInClass.pop();
	for (int i = 0; i < testNumber - 1; i++) {
		if (builtInClass.empty())
			emptyHeapErrors++;
		if (builtInClass.top().length < heapMin)
			outOfOrderErrors++;
		
		heapMin = builtInClass.top().length;
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
		testEdges.push_back(Edge(0, 0, rand(gen)));
		builtInClass.push(testEdges.back());
	}
	size--;
	heapMin = builtInClass.top().length;
	builtInClass.pop();
	outOfOrderErrors = 0;
	while (!builtInClass.empty()) {
		size--;
		if (builtInClass.top().length < heapMin)
			outOfOrderErrors++;
		builtInClass.pop();
	}
	if (outOfOrderErrors > 0)
		std::cout << outOfOrderErrors << " edges returned out of order!" << std::endl;
	if (size != 0) {
		std::cout << "Error!  Edges pushed != Edges popped!" << std::endl;
	}
}

void NavigationMesh::Draw(DefaultShader shader)
{
	if (mesh) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		mesh->Draw(shader);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
}

EdgeHeap::EdgeHeap()
{
	heap.reserve(defaultSize);
	heap.push_back(NULL); //fill up spot zero
}

EdgeHeap::EdgeHeap(std::vector<Face*>& faces)
{
	heap.reserve(2 * faces.size() + 1);
	heap.push_back(NULL); //fill up spot zero
	EdgeSet edges;

	for (unsigned int i = 0; i < faces.size(); i += 3) {
		Face *face = faces[i];
		glm::vec3 a = face->vertex1;
		glm::vec3 b = face->vertex2;
		glm::vec3 c = face->vertex3;

		edges.insert(new EdgeContainer(face->index1, face->index2, a, b));
		edges.insert(new EdgeContainer(face->index2, face->index3, b, c));
		edges.insert(new EdgeContainer(face->index3, face->index1, c, a));
	}

	// Insert the unique edges to a heap
	for (EdgeContainer *e : edges.edges) {
		push(new Edge(e->indexA, e->indexB, e->len));
	}
}

EdgeHeap::~EdgeHeap()
{

}

bool EdgeHeap::empty()
{
	return heap.size() == 1;
}

Edge * EdgeHeap::pop()
{
	// This method is O(2logn).  
	// An O(logn) method is to promote children until you reach a leaf,
	// then filling that leaf with another leaf and bubbling it up.
	Edge *topEdge = heap[1];
	Edge *bubbleEdge = heap.back();
	heap[1] = bubbleEdge;
	heap.pop_back();

	int index = 1;
	int leftChild = 2;
	
	while (leftChild < heap.size()) {
		float minValue = bubbleEdge->length;
		int minIndex = index;

		float childValue = heap[leftChild]->length;
		if (childValue < minValue) {
			minValue = childValue;
			minIndex = leftChild;
		}
		
		int rightChild = leftChild + 1;
		if (rightChild < heap.size()) {
			childValue = heap[rightChild]->length;
			if (childValue < minValue) {
				minValue = childValue;
				minIndex = rightChild;
			}
		}

		if (minIndex == index)
			break;

		heap[index] = heap[minIndex];
		heap[minIndex] = bubbleEdge;
		index = minIndex;
		leftChild = index << 1;
	}

	return topEdge;
}

void EdgeHeap::push(Edge * e)
{
	int index = heap.size();
	heap.push_back(e);
	int parent = index >> 1; // Divide by two

	if (parent > 0) {
		// Can this be threaded?  It's overkill, but if it lets
		// the caller start faster then it might be worth it.
		Edge *swapHolder = heap[parent];
		float insertValue = e->length;
		float parentValue = heap[parent]->length;

		while (insertValue < parentValue) { // i/2 is equivalent to floor(i/2)
			heap[index] = swapHolder;
			heap[parent] = e;

			index = parent;
			// If index is at the top of the heap (ie at 1):
			// Let parentValue = insertValue so the loop breaks
			// Alternately you could branch and break, but that's more inside the loop
			if (index > 1) 
				parent >>= 1;

			swapHolder = heap[parent];
			parentValue = swapHolder->length;
		}
	}
}

bool EdgeHeap::heapTest()
{
	std::default_random_engine gen;
	std::uniform_real_distribution<double> rand(0.0, 1000.0);
	std::vector<Edge*> testEdges;
	int testNumber = 10000;

	testEdges.reserve(testNumber * 1.5);
	for (int i = 0; i < testNumber; i++) {
		testEdges.push_back(new Edge(0, 0, rand(gen)));
	}

	if (!empty()) {
		std::cout << "Error!  Edgeheap not empty!" << std::endl;
		return false;
	}

	for (Edge *e : testEdges) {
		push(e);
	}
	if (empty()) {
		std::cout << "Error!  Edgeheap empty!" << std::endl;
		return false;
	}

	float heapMin = pop()->length;
	for (int i = 0; i < testNumber - 1; i++) {
		if (empty()) {
			std::cout << "Error!  Edgeheap empty!" << std::endl;
			return false;
		}
			
		Edge *e = pop();
		if (e->length < heapMin) {
			std::cout << "Error!  Edges returned out of order!" << std::endl;
			return false;
		}

		heapMin = e->length;
	}
	if (!empty()) {
		std::cout << "Error!  Edgeheap not empty!" << std::endl;
		return false;
	}
		
	int size = 0;
	for (Edge *e : testEdges) {
		size++;
		push(e);
	}
	for (int i = 0; i < testNumber / 2; i++) {
		size--;
		pop();
	}
	for (int i = 0; i < testNumber / 2; i++) {
		size++;
		testEdges.push_back(new Edge(0, 0, rand(gen)));
		push(testEdges.back());
	}
	size--;
	heapMin = pop()->length;
	while (!empty()) {
		size--;
		Edge *e = pop();
		if (e->length < heapMin) {
			std::cout << "Error!  Edges returned out of order!" << std::endl;
			return false;
		}
	}
	if (size != 0) {
		std::cout << "Error!  Edges pushed != Edges popped!" << std::endl;
		return false;
	}

	for (Edge *e : testEdges) {
		delete e;
	}
	return true;
}