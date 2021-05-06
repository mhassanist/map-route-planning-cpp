#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    m_Model = model;  

    start_node  =  &m_Model.FindClosestNode(start_x,start_y);
    end_node    =  &m_Model.FindClosestNode(end_x,end_y);

    start_node->g_value = start_node->distance(*start_node);
    start_node->h_value = CalculateHValue(start_node);
}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
   return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    current_node->FindNeighbors();
    
    for (auto *node : current_node->neighbors)
    {
        if(node->visited) continue;

        node->parent = current_node;
        
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->h_value = CalculateHValue(node);
        
        node->visited = true;
        
        this-> open_list.push_back(node);
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(std::begin(open_list), std::end(open_list),
        [](RouteModel::Node* a, RouteModel::Node* b) 
        { return a->g_value+a->h_value > b->g_value+b->h_value; });

        RouteModel::Node *top = open_list.back();
        open_list.pop_back();
    
    return top;
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
    
    RouteModel::Node *temp = current_node;

    while (temp->parent != nullptr)
    {
        path_found.push_back(*temp);
        distance += temp->distance(*temp->parent);
        
        temp = temp->parent;
    }
    path_found.push_back(*temp);

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
    
    current_node            = start_node;
    current_node->visited   = true;
    current_node->h_value   = CalculateHValue(current_node);
    current_node->g_value   = 0;

    this->open_list.push_back(current_node);
    

    while (open_list.size()>0)
    {   
        current_node =  NextNode();
        if (current_node == end_node)
            break;        
            
        AddNeighbors(current_node);

    }

    m_Model.path = ConstructFinalPath(end_node);

}