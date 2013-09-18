/*
 * frontier_exploration.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 * This class aims at providing a list of frontiers based on the cuurent
 * knowledge of the environment to perform exploration.
 *
 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-06
 * license: BSD
 */
#ifndef FRONTIER_EXPLORATION_HPP
#define FRONTIER_EXPLORATION_HPP

#include <vector>
#include <ostream>

#include "gladys/point.hpp"
#include "gladys/nav_graph.hpp"

// NOTE : it currently work only with 2D points

namespace gladys {

/*{{{ f_atttributes class
 ****************************************************************************/
class f_attributes {

public:
    /* the frontier attributes */
    // NB: the attributes of a frontier are dependent from others' attributes…
    double ID;                  // ID of the frontier (= its position in the vector)
    double size;                // nbr of frontier points
    double ratio ;              // importance of the frontier among others 
                                // ( max = 1  ; "value < 0" <=> unknown)
    //double distance;            // some distance between XX and the frontier
    //unsigned int proximity ;    // some distance between XX and the frontier
    
    /* operators */
    bool operator> (const f_attributes& f) const {//{{{
        return ( size > f.size );
    }//}}}

};

std::ostream& operator<< (std::ostream &out, const f_attributes& f) {//{{{
    out << "{ #" << f.ID << ": size = " << f.size << "; ratio = " << f.ratio << " }";
    return out;
}///}}}

//}}}

/*{{{ frontier_detector class
 ****************************************************************************/

/* frontier_detector class */
class frontier_detector {

private :
    /* internal data */
    const nav_graph& ng ;                       // Navigation Graph, used as model of the map
    std::vector< points_t > frontiers ;         // the list of the frontiers
    std::vector< f_attributes > attributes ;    // the frontiers attributes

    /* hidden computing functions */
    /** compute_frontiers_WFD
     *
     * Compute the frontiers with the classical WFD (Wavefront Frontier
     * Detector) algorithm. Result is stored in frontiers.
     *
     * @param seed : the seed for the wavefront detection (usually it is the
     * robot position) ; Note that the seed must be in the "known" area.
     *
     * @throws : throw an exception if the seed is not valid.
     *
     */
    void compute_frontiers_WFD( const point_xy_t &seed );

    /** compute_attributes
     *
     * Compute the frontier attributes for each elements in the frontiers list
     * frontiers.
     *
     * @param seed : the seed for the wavefront detection (usually it is the
     * robot position) ; Note that the seed must be in the "known" area.
     *
     */
    void compute_attributes( const point_xy_t &seed );

    /** is_frontier
     *
     * Tell if the given vertex is a vertex frontier. To be considered so it
     * must not have any 'unknown' edge, but should have an adjacent vertex that
     * has one.
     *
     * @param p : the vertex which is tested.
     *
     */
    bool is_frontier( vertex_t p ) const ;

public:
    /* Name of the available algorithms to compute frontiers */
    typedef enum {WFD, FFD} algo_t;

    /** frontier_detector constructor
     *
     * Constuctor using a pre-existant weight_map.
     *
     * @param nav_graph : a previously computed graph for navigation (nav_graph)
     *
     */
    frontier_detector( const nav_graph& _ng ) ;

    /* hidden computing functions */
    /** compute_frontiers
     *
     * Compute the frontiers with the given algorithm and parameters.
     * Sort the frontiers list by the size of the frontiers (descending order).
     *
     * @param seed : the seed for the wavefront detection (usually it is the
     * robot position) ; Note that the seed must be in the "known" area.
     *
     * @param algo : chose the algorithm used to compute the frontiers ; default
     * is WFD ( Wavefront Frontier Detection).
     *
     * @throws : throw an exception if the algo provided is invalid.
     *
     */
    void compute_frontiers(const point_xy_t &seed, algo_t algo = WFD);

    //void save_frontiers(const std::string& filepath) ;

    /* getters */
    const nav_graph& get_graph() const {//{{{
        return ng;
    }//}}}
    const std::vector< points_t >& get_frontiers() const {//{{{
        return frontiers;
    }//}}}
    const std::vector< f_attributes >& get_attributes() const {//{{{
        return attributes;
    }//}}}

};//}}}

} // namespace gladys

#endif // FRONTIER_EXPLORATION_HPP

