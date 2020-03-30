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

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		helper_insert(&root, 0, point, id);
	}

	void helper_insert(Node **node, uint depth, std::vector<float> point, int id)
	{
		uint index = depth % point.size();
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else if(point[index] < (*node)->point[index])
		{
			helper_insert(&(*node)->left, depth+1, point, id);
		}
		else
		{
			helper_insert(&(*node)->right, depth+1, point, id);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		helper_search(target, root, 0, distanceTol, ids);
		return ids;
	}

	void helper_search(std::vector<float> target, Node *node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node == NULL) return;

		bool isNearby = true;
		for(int i = 0; i < target.size(); i++)
			isNearby = isNearby &&
				node->point[i] >= (target[i] - distanceTol) && 
				node->point[i] <= (target[i] + distanceTol);

		//if(node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol) && 	
		//	node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol))
		if(isNearby)
		{
			// float sqr_distance = /*sqrt*/((node->point[0] - target[0]) * (node->point[0] - target[0]) + 
			// 					(node->point[1] - target[1]) * (node->point[1] - target[1]));

			float sqr_distance = 0; 
			for(int i = 0; i < target.size(); i++)
			{
				sqr_distance += (node->point[i] - target[i]) * (node->point[i] - target[i]);
			}

			if(sqr_distance <= distanceTol*distanceTol)
				ids.push_back(node->id);
		}

		uint index = depth % target.size();
		if((target[index] - distanceTol) < node->point[index])
			helper_search(target, node->left, depth+1, distanceTol, ids);
		if((target[index] + distanceTol) > node->point[index])
			helper_search(target, node->right, depth+1, distanceTol, ids);
	}
};




