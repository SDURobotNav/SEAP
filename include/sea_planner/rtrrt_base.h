#ifndef RTRRT_BASE_H_
#define RTRRT_BASE_H_

#include <vector>
#include <geometry_msgs/Point.h>
#include "utility.h"

namespace rtrrt_ns{
    enum NodeState{
        ROOT=0,
        NORMAL=1,
        LEAF=2,
        OBSTACLE=3,
    };

    // RTRRT node
    struct Node{
        geometry_msgs::Point state;             // Node position
        
        Node *parent=NULL, *prevParent=NULL;    // Parent node
        struct Edge *endEdge;                          // The Edge connected with parant node
        std::list<struct Edge*> startEdges;            // The Edge connected with child node      
        std::list<Node*> children;              // Child nodes
        int obstacle_orientation[8] = {0, 0, 0, 0, 0, 0, 0, 0};    // This array is used to certify obstacle node, if element num(=1) > threshold,
                                        //  this node is considered as obstacle

        float disToRoot;                        // The sum of branch length from root to this node in tree
        float gradientToRoot;                  // The sum of gradient from root to this node in tree
        float penaltyFactor = 1.0;
        NodeState nodeState;                    // Node state: ROOT, NORMAL, LEAF, OBSTACLE
        
        Node(){}
        // Init Func
        Node(float x_, float y_,float z_, float disToStart_, float gradientToRoot_, Node* parent_ = NULL, NodeState nodeState_= NodeState::NORMAL)
        {
            state.x = x_;
            state.y = y_;
            state.z = z_;
            disToRoot = disToStart_;
            gradientToRoot = gradientToRoot_;
            parent = parent_;
            nodeState = nodeState_;
        }

    };
    // RTRRT edge
    struct Edge{
        struct Node *fromNode;               // Parent Node 
        struct Node *toNode;                 // Child Node
        float length;                        // The length of edge
        float elevation_diff;                // The difference of Z value
        float gradient;                      // The gradient of Edge, rad form
        
        Edge(){};
        // Init
        Edge(Node* fromNode_, Node *toNode_){
            fromNode = fromNode_;
            toNode = toNode_;
            length = util::distance(fromNode_->state.x, toNode_->state.x,
                                    fromNode_->state.y, toNode_->state.y);
                                 
        }
        bool operator == (const Edge & edge){
            if (this->fromNode == edge.fromNode &&
                this->toNode == edge.toNode)
            {
                return true;
            }
            return false;
        }
    };

    typedef struct Node Node;
    typedef struct Edge Edge;

    // inline bool rtrrt_ns::Edge::operator==(const rtrrt_ns::Edge& edge) const
    // {
    //     if (this->fromNode == edge.fromNode &&
    //         this->toNode == edge.toNode)
    //     {
    //         return true;
    //     }
    //     return false;
    // }

}

struct iKdTreeNode{
    float x;
    float y;
    float z;
    rtrrt_ns::Node *node;
    iKdTreeNode(){};
    iKdTreeNode(rtrrt_ns::Node *node_){
        x = node_->state.x;
        y = node_->state.y;
        z = 0.0;
        node = node_;
    }
};

#endif  // RTRRT_BASE_H_