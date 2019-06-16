/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
	{
		std::vector<float> point;
		int id;
		Node* left;
		Node* right;

		Node(std::vector<float> arr, int setId)
		:	point(arr), id(setId), left(NULL), right(NULL)
		{}
	};

struct KdTree
{
	Node* root;
	int dimension;

	KdTree(int dim = 3)
	: root(NULL), dimension(dim)
	{}

	void _insert(Node *&node, std::vector<float> point, int id, int depth) {
		if (node == NULL) {
			node = new Node(point, id);
		} else {
			uint dim = depth % this->dimension;
			if (node->point[dim] > point[dim]) {
				_insert(node->left, point, id, depth+1);
			} else {
				_insert(node->right, point, id, depth+1);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		_insert(root, point, id, 0);
	}

	bool checkOverlapWithBox(const std::vector<float> target, const std::vector<float> point, float distanceTol) {
		bool inside = true;
		for (int dim = 0; dim < this->dimension; dim++) {
			inside = inside && (target[dim] - distanceTol <= point[dim]) && (target[dim] + distanceTol >= point[dim]);
		}
		return inside;
	}

	bool checkDistance(const std::vector<float> target, const std::vector<float> point, float distanceTol) {
		float distance = 0.0;
		for (int dim = 0; dim < this->dimension; dim++) {
			distance += std::pow(target[dim] - point[dim], 2);
		}
		return std::sqrt(distance) <= distanceTol;
	}

	void _search(std::vector<int> &ids, const Node *node, std::vector<float> target, float distanceTol, int depth) {
		if (node != NULL) {
			uint dim = depth % this->dimension;
			if (checkOverlapWithBox(target, node->point, distanceTol)) {
				if (checkDistance(target, node->point, distanceTol)) {
					ids.push_back(node->id);
				}
			}
			if (target[dim] - distanceTol < node->point[dim]) {
				_search(ids, node->left, target, distanceTol, depth+1);
			}
			if (target[dim] + distanceTol > node->point[dim]) {
				_search(ids, node->right, target, distanceTol, depth+1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		_search(ids, root, target, distanceTol, 0);
		return ids;
	}
};
