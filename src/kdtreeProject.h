/* \author Aaron Brown */
// Quiz on implementing kd tree

// Project: Switch from PCL algorithm to KD-Tree Clustering
//#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	// Project: Switch from PCL algorithm to KD-Tree Clustering
	pcl::PointXYZI point;
	//std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	// Project: Switch from PCL algorithm to KD-Tree Clustering
	Node(pcl::PointXYZI arr, int setId)
	//Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	//~Node()
	//{
	//	delete left;
	//	delete right;
	//}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	//~KdTree()
	//{
	//	delete root;
	//}

	// Lesson 3, Chapter 6
	// Recursive Helper Function
	// Project: Switch from PCL algorithm to KD-Tree Clustering
	void insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id){
	//void insertHelper(Node** node, uint depth, std::vector<float> point, int id){
		if (*node == NULL){ // Tree is empty.
			*node = new Node(point, id);
		}
		else{ // Tree is not empty.
			// Calculate current dim
			// Project: Switch from PCL algorithm to KD-Tree Clustering
			// 3 dimensions
			uint cd = depth % 3;
			if (cd==0){
				if (point.x < (*node)->point.x){
					insertHelper(&((*node)->left), depth+1, point, id);
				}
				else {
					insertHelper(&((*node)->right), depth+1, point, id);
				}
			}
			else if (cd==1){
				if (point.y < (*node)->point.y){
					insertHelper(&((*node)->left), depth+1, point, id);
				}
				else {
					insertHelper(&((*node)->right), depth+1, point, id);
				}
			}
			else {
				if (point.z < (*node)->point.z){
					insertHelper(&((*node)->left), depth+1, point, id);
				}
				else {
					insertHelper(&((*node)->right), depth+1, point, id);
				}
			}
			//uint cd = depth % 2;
			// Traverse Tree
			//if (point[cd] < ((*node)->point[cd])){
			//	insertHelper(&((*node)->left), depth+1, point, id);
			//}
			//else {
			//	insertHelper(&((*node)->right), depth+1, point, id);
			//}
		}
	}
	
	// Project: Switch from PCL algorithm to KD-Tree Clustering
	void insert(pcl::PointXYZI point, int id)
	//void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		// Lesson 3, Chapter 6
		insertHelper(&root, 0, point, id); // recursive helper

	}

	// Lesson 3, Chapter 7
	// Project: Switch from PCL algorithm to KD-Tree Clustering
	void searchHelper(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	//void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL){
			// Project: Switch from PCL algorithm to KD-Tree Clustering
			// Temporary variables: 3 dimensions
			float deltaX = node->point.x - target.x;
			float deltaY = node->point.y - target.y;
			float deltaZ = node->point.z - target.z;

			// Check whethere inside Target Box
			if ((deltaX >= -distanceTol && deltaX <= distanceTol) &&
				(deltaY >= -distanceTol && deltaY <= distanceTol) &&
				(deltaZ >= -distanceTol && deltaZ <= distanceTol)){
				// Calculate distance only for the points inside target box to reduce calculation cost
				float distance = sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);
				if (distance <= distanceTol){
					ids.push_back(node->id);
				}
			}

			// Check accross boundary: 3 dimensions
			uint cd = depth % 3;

			if (cd == 0){
				if (deltaX > -distanceTol){
					searchHelper(target, node->left, depth+1, distanceTol, ids);
				}
				if (deltaX < distanceTol){
					searchHelper(target, node->right, depth+1, distanceTol, ids);
				}
			}
			else if (cd == 1){
				if (deltaY > -distanceTol){
					searchHelper(target, node->left, depth+1, distanceTol, ids);
				}
				if (deltaY < distanceTol){
					searchHelper(target, node->right, depth+1, distanceTol, ids);
				}
			}
			else {
				if (deltaZ > -distanceTol){
					searchHelper(target, node->left, depth+1, distanceTol, ids);
				}
				if (deltaZ < distanceTol){
					searchHelper(target, node->right, depth+1, distanceTol, ids);
				}
			}			
			// Check whethere inside Target Box
			//if ( (node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol)) && (node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol))){
			//	float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
			//	if(distance <= distanceTol){
			//		ids.push_back(node->id);
			//	}
			//}
			// Check accross boundary
			//if((target[depth%2]-distanceTol) < node->point[depth%2]){
			//	searchHelper(target, node->left, depth+1, distanceTol, ids);
			//}
			//if((target[depth%2]+distanceTol) > node->point[depth%2]){
			//	searchHelper(target, node->right, depth+1, distanceTol, ids);
			//}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	// Project: Switch from PCL algorithm to KD-Tree Clustering
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	//std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// Lesson 3, Chapter 7
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};




