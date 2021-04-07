#include "route_planner.h"
#include <algorithm>
#include <math.h>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Calculating h value using
    float h_value = 0.0f;
    h_value = node->distance(*end_node);

    return h_value;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	//Finds neighbour and adds to neighbour
    current_node->FindNeighbors();
    current_node->visited = true;

    for (auto cur_node: current_node->neighbors)
    {
        cur_node->parent = current_node;												// assigns parent to each node
        cur_node->h_value = CalculateHValue(cur_node);									// calculates h value
        cur_node->g_value = current_node->g_value + current_node->distance(*cur_node);	// calculates g value
      	cur_node->visited = true;														// marks the node as visited
        open_list.push_back(cur_node);													// adds the node to open list
    }
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
	
  	// sorts the open list based on the lambda expression
  	// Ref: https://docs.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-160
    sort(open_list.begin(), open_list.end(), [](const RouteModel::Node* node_a, const RouteModel::Node* node_b){
        return (node_a->g_value + node_a->h_value) < (node_b->g_value + node_b->h_value);
    });

  	// choose the node with minimum f value
    RouteModel::Node *node = open_list[0];
    open_list.erase(open_list.begin());				// remove the selected node from the list

    return node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    path_found.push_back(*current_node);									// add the current node to path_found

    while(current_node->parent != nullptr)
    {
        distance = distance + current_node->distance(*current_node->parent);// update the distance after each iteration
        current_node = current_node->parent;
      	path_found.insert(path_found.begin(), *current_node);				// adds the node to the beginning of the vector
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;

    // TODO: Implement your solution here
    AddNeighbors(current_node);								// adds the neighbours of current_node to the open_list
  	current_node = NextNode();								// returns the next node

    while (!open_list.empty())								// iterating over the open list
    {
        if (current_node->distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(end_node);	// final path is returned if found
            return;
        }
        AddNeighbors(current_node);
        current_node = NextNode();
    }

}