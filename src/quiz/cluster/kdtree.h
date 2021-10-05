/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node_quiz
{
	std::vector<float> point;
	int id;
	Node_quiz* left;
	Node_quiz* right;

	Node_quiz(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	//~Node()
	//{
	//	delete left;
	//	delete right;
	//}
};

struct KdTree_quiz
{
	Node_quiz* root;

	KdTree_quiz()
	: root(NULL)
	{}

	//~KdTree()
	//{
	//	delete root;
	//}

	// Lesson 3, Chapter 6
	// Recursive Helper Function
	void insertHelper(Node_quiz** node, uint depth, std::vector<float> point, int id){
		if (*node == NULL){ // Tree is empty.
			*node = new Node_quiz(point, id);
		}
		else{ // Tree is not empty.
			// Calculate current dim
			uint cd = depth % 2;
			// Traverse Tree
			if (point[cd] < ((*node)->point[cd])){
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			else {
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}
	

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		// Lesson 3, Chapter 6
		insertHelper(&root, 0, point, id); // recursive helper

	}

	// Lesson 3, Chapter 7
	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL){
			// Check whethere inside Target Box
			if ( (node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol)) && (node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol))){
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
				if(distance <= distanceTol){
					ids.push_back(node->id);
				}
			}
			// Check accross boundary
			if((target[depth%2]-distanceTol) < node->point[depth%2]){
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			if((target[depth%2]+distanceTol) > node->point[depth%2]){
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// Lesson 3, Chapter 7
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};




