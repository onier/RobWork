#ifndef CNODEPOOL_HPP_
#define CNODEPOOL_HPP_

#include "ConstraintNode.hpp"
#include "ConstraintEdge.hpp"

#include <vector>
#include <list>
#include <stack>

namespace dynamics {

/**
 * @brief interface for creating and deleting constraintEdges and ConstraintNodes.
 * ConstraintEdges are frequently created and deleted so efficient data structures
 * are here needed.
 */

class CNodePool {

public:
    /**
     * @brief initialize the node and edge buffers
     */
    CNodePool(int nrNodes=0, int nrEdges=0);

    /**
     * @brief create a ConstraintNode
     */
    ConstraintNode *createCNode(ConstraintNode::NodeType type);

    /**
     * @brief delete a constraint node
     */
    void deleteCNode(ConstraintNode* node);


    ConstraintEdge *createCEdge(const CNodePair& pair, ConstraintEdge::EdgeType type);

    /**
     * @brief delete an edge from the pool. The edge will be recycled
     *
     */
    void deleteCEdge(ConstraintEdge* edge);

    /**
     * @brief gets the complete list of constraint nodes
     * in the pool. NULL elements can occour.
     */
    const std::vector<ConstraintNode*>& getNodes() const ;

    /**
     * @brief gets the complete list of constraint edges
     * in the pool. NULL elements can occour.
     */
    const std::vector<ConstraintEdge*>& getEdges() const ;

protected:
    std::vector<ConstraintNode*> _nodes;
    std::vector<ConstraintEdge*> _edges;
    std::stack<int> _freeNodeIDs;
    std::stack<int> _freeEdgeIDs;
};

}

#endif /*CNODEPOOL_HPP_*/
