/*
 * frontier_exploration.cpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 * This class aims at providing a list of frontiers based on the cuurent
 * knowledge of the environment to perform exploration.

 * author:  Cyril Robin <cyril.robin@laas.fr>
 * created: 2013-09-06
 * license: BSD
 */

//#include <string>
//#include <fstream>    // file stream
#include <ostream>      // output stream
#include <stdexcept>    // exceptions
#include <deque>
#include <vector>

//#include <boost/graph/adjacency_list.hpp>

#include "gladys/frontier_exploration.hpp"

#ifndef HEIGHT_CONNEXITY
#define HEIGHT_CONNEXITY
#endif

namespace gladys {

/*{{{ frontier_detector class
 ****************************************************************************/

    /* constructors */
    frontier_detector::frontier_detector(const nav_graph& _ng) : ng(_ng) {}

    /* computing functions */
    void frontier_detector::compute_frontiers_WFD(const point_xy_t &seed) {//{{{
        graph_t g = ng.get_graph() ;

        /* get the seed in the graph (related vertex) */
        vertex_t s = ng.get_closest_vertex( seed );
        // no need to check the position : this is delegated to the graph
        // consistency...

        /* {{{ compute frontiers with the WFD algorithm
         *
         * Use an adaptation of the description given by :
         * "Robot Exploration with Fast Frontier Detection : Theory and
         * Experiments", M. Keidar afd G. A. Kaminka (AAMAS 2012)
         *
         */
        // Requested containers
        // queues
        std::deque< vertex_t > m_queue ;       // std::queue are restricted std::deque
        std::deque< vertex_t > f_queue ;       // std::queue are restricted std::deque

        // markers lists
        std::vector< bool > open_map       (boost::num_vertices(g), false) ;
        std::vector< bool > close_map      (boost::num_vertices(g), false) ;
        std::vector< bool > open_frontier  (boost::num_vertices(g), false) ;
        std::vector< bool > close_frontier (boost::num_vertices(g), false) ;

        // points
        vertex_t p,q ;

        //init
        frontiers.empty();    // clear the previous frontiers
        m_queue.push_back( s );
        //open_map[ ng.get_map().idx( g[s].pt )] = true ;
        open_map[ s ] = true ;

        int c1 = 0;
        // Main while over queued map points
        while ( !m_queue.empty() ) {

            p = m_queue.front();
            m_queue.pop_front();

            // if p has already been visited, then continue
            if ( close_map[ p ] ) {
                continue ;
            }

            // else, if p is a frontier point,
            // compute the whole related frontier
            //if ( is_frontier( p ) ) {
            if ( ng.has_unknown_edge( p ) ) {
                int c2 = 0 ;

                f_queue.clear();
                // create a new frontier
                vertices_t frontier_vertices ;

                f_queue.push_back( p );
                open_frontier[ p ] = true ;

               // while over potential frontier points
               while ( !f_queue.empty() ) {

                   q = f_queue.front();
                   f_queue.pop_front();

                   // if q has already been visited, then continue
                   if  ( close_map[ q ] || close_frontier[ q ])
                       continue;

                    // if p is a frontier point,
                    // deal with it and its neighbours
                    //if ( is_frontier( q ) ) {
                    if ( ng.has_unknown_edge( q ) ) {
                        frontier_vertices.push_back( q );
                        //for all neighbours of q
                        for ( auto i : ng.get_neighbours( q ) ) {
                            // if NOT marked yet
                            if  ( !( close_map[ i ] || close_frontier[ i ]
                            || open_frontier[ i ])) {
                            // then proceed
                                f_queue.push_back( i );
                                open_frontier[ i ] = true ;
                            }
                        }
                    }
                    // mark q
                    close_frontier[ q ] = true ;
                }

                // create a new frontier
                frontiers.push_back( points_t() );

                // mark all points of the new frontier in the closed list
                // and effectively save them in the new frontier
               for ( auto i : frontier_vertices ) {
                    close_map[ i ] = true ;
                    frontiers.back().push_back( g[i].pt );
               }
           }

           //for all neighbours of p
           for ( auto i : ng.get_neighbours( p ) ) {
               // if NOT marked yet
               if  ( !( close_map[ i ] || open_map[ i ])
               // and is in the "Open Space" (not unknown nor an obstacle)
               //&& ( ! (data[ i ]  < 0 || data[ i ] == HUGE_VALF ))) {
               // HAS_ALL_EDGE_AS_UNKNOWN => vertex = uknown
               && !ng.is_unknown( i ) ) {
                   // then proceed
                   m_queue.push_back( i );
                   open_map[ i ] = true ;
               }
           }

           //mark p
           close_map[ p ] = true ;
        }
        //}}}
    }//}}}

    void frontier_detector::compute_frontiers(const point_xy_t &seed, algo_t algo ){//{{{
        // try running the algo
        switch(algo) {
            case WFD : // Wavefront Frontier Detection
                compute_frontiers_WFD( seed ) ;
                break;
            case FFD : // Fast Frontier Detection
                throw  std::runtime_error("Fast Frontier Detection is not imlemented yet");
                break;
            default : // Unknown  algorithm
                throw  std::runtime_error("Unknown algorithm for frontier detection");
                break;
        }

        // compute the frontiers attributes
        compute_attributes( seed );

        // sort the frontiers attributes by the size criteria
        std::sort( attributes.begin(), attributes.end(), std::greater<f_attributes>() ); // descending order

    }//}}}

    void frontier_detector::compute_attributes( const point_xy_t &seed ) {//{{{
        /* init */
        size_t total_fPoints = 0 ;
        attributes.resize( frontiers.size() ) ;

        /* loop over the frontiers list */
        for ( unsigned int i = 0 ; i < frontiers.size() ; i++ ) {
            attributes[i].ID = i ;
            attributes[i].size = frontiers[i].size() ;
            total_fPoints += frontiers[i].size() ;
        }

        for ( auto& a : attributes )
            a.ratio = (double) a.size / (double) total_fPoints ;
    }//}}}

    //void frontier_detector::save_frontiers( const std::string& filepath ) {
    //}

//}}}

} // namespace gladys
