#include "route_planner.h"
#include <algorithm>

// Helper function for sorting nodes
bool CompareNodes(const RouteModel::Node *a, const RouteModel::Node *b) {
    // sort by highest to lowest f-value first
    float dist_a = (a->g_value + a->h_value);
    float dist_b = (b->g_value + b->h_value);
    return ( dist_a > dist_b );
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    //
    // Initial implementation (7/11/2020): Appears that FindClosestNode returns a node reference.
    //  RoutePlanner stores a pointer to the closest node.
    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));

}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
//
// Initial implementation (7/11/2020): Not totally sure I'm dereferencing things correctly.
//  RoutePlanner start_node and end_node are pointers. RouteModel::Node.distance() expects
//  a node Object. Generally, distance() computes the euclidean distance based on some node
//  member variable x and y, but I don't see these in the definition of Node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h = (*node).distance(*end_node);
    return h;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for(RouteModel::Node *neighbor: current_node->neighbors) {
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->parent = current_node;
        neighbor->visited = true;
        this->open_list.push_back(neighbor);
    }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // sort node list from highest f-value to lowest. Therefore get last node for min f-value
    std::sort(this->open_list.begin(), this->open_list.end(), CompareNodes);
    RouteModel::Node *next_node = this->open_list.back();
    this->open_list.pop_back();
    return next_node;
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
    while(current_node != this->start_node){
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }

    path_found.push_back(*current_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    current_node = this->start_node;
    current_node->visited = true;

    // TODO: Implement your solution here.
    this->AddNeighbors(current_node);
    while(!this->open_list.empty()){
        current_node = this->NextNode();
        current_node->visited = true;
        if(current_node == this->end_node){
            break;
        }

        this->AddNeighbors(current_node);
    }

    m_Model.path = this->ConstructFinalPath(current_node);


}