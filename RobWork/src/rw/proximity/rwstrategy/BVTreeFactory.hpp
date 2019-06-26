/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RW_PROXIMITY_BVTREEFACTORY_HPP_
#define RW_PROXIMITY_BVTREEFACTORY_HPP_


#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/TriMesh.hpp>


#include <rw/common/macros.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <rw/common/Timer.hpp>

#include <float.h>

#include <rw/geometry/BV.hpp>
#include <rw/geometry/OBB.hpp>
#include <rw/geometry/OBBFactory.hpp>
#include <rw/geometry/IndexedTriArray.hpp>
#include <rw/geometry/analytic/GenericFace.hpp>
#include <rw/geometry/analytic/Shell.hpp>
#include <rw/geometry/analytic/IndexedFaceArray.hpp>

#include "BinaryBVTree.hpp"

namespace rw {
namespace proximity {



	/**
	 * @brief factory for creating bounding volume trees.
	 *
	 */
	class BVTreeFactory {
	public:

		/**
		 * @brief interface of bounding volume splitting strategy
		 */
		template<class BV>
		struct BVSplitterStrategy {
			virtual size_t partitionMesh(rw::geometry::IndexedTriArray<>& mesh, const BV& bv) const = 0;
		};

		/**
		 * @brief Interface for bounding volume splitting strategy for objects
		 * with analytical surfaces.
		 */
		template<class BV>
		struct BVShellSplitterStrategy {
		        /**
		         * @brief Split the shell in two.
		         *
		         * @param shell [in/out] the shell to split.
		         * @param bv [in] the bounding volume of the \b shell.
		         * @return the quality of the split. A good split returns a
		         * large value.
		         */
		        virtual size_t partitionShell(rw::geometry::IndexedFaceArray& shell, const BV& bv) const = 0;
		};

		template<class T>
        struct TriMeshAccessor: public PrimArrayAccessor<rw::geometry::Triangle<T> > {

            TriMeshAccessor(rw::geometry::TriMesh::Ptr mesh):_mesh(mesh){}

            void getPrimitive(size_t triidx, rw::geometry::Triangle<T>& dst) const {
                _mesh->getTriangle(triidx, dst);
            }

            size_t getSize() const{
                return _mesh->getSize();
            }

        private:
            rw::geometry::TriMesh::Ptr _mesh;
        };

		//! @brief Access GenericFace primitive of Shell.
		template<class T>
		struct ShellAccessor: public PrimArrayAccessor<rw::geometry::GenericFace> {
		        /**
		         * @brief Constructor.
		         * @param object [in] the shell object.
		         */
		        ShellAccessor(rw::geometry::Shell::CPtr object):_object(object) {}

		        //! @copydoc PrimArrayAccessor::getPrimitive
		        void getPrimitive(size_t faceIdx, rw::geometry::GenericFace& face) const {
		            _object->getFace(faceIdx, face);
		        }

		        //! @copydoc PrimArrayAccessor::getSize
		        size_t getSize() const{
		            return _object->size();
		        }

		    private:
		        rw::geometry::Shell::CPtr _object;
		};

		/**
		 * @brief creates a splitter strategy for OBB tree construction using
		 * an object median splitting strategy.
		 *
		 * splits a trimesh in one of the axis of a bounding volume. The splitter use
		 * a median based strategy where the splitpoint is determined as the object median
		 * and as such is suitable for creating balanced OBB trees.
		 * Median is found in O(n log n) time
		 */
        template<class OBV>
		rw::common::Ptr<BVSplitterStrategy<OBV> > makeOBVMedianSplitter(){
		    return rw::common::ownedPtr( new OBVMedianSplitter<OBV>() );
		}

        /**
         * @brief Creates a splitter strategy for OBB tree construction using
         * an object median splitting strategy.
         *
         * Splits a shell representation with analytic surfaces in one of the
         * axes of the bounding volume. The splitter use a median based
         * strategy where the splitpoint is determined as the object median
         * and as such is suitable for creating balanced OBB trees.
         *
         * @return the splitting strategy.
         */
        template<class OBV>
        rw::common::Ptr<BVShellSplitterStrategy<OBV> > makeOBVShellMedianSplitter()
        {
                return rw::common::ownedPtr( new OBVShellMedianSplitter<OBV>() );
        }

		/**
		 * @brief creates a splitter strategy for OBB tree construction using
		 * a spatial median splitting strategy.
		 *
		 * splits a trimesh in one of the axis of a bounding volume such that the bounding
		 * volume is split in two equal parts. Median is found in constant time.
		 */
        template<class OBV>
		rw::common::Ptr<BVSplitterStrategy<OBV > > makeOBVSpatialMedianSplitter(){
		    return rw::common::ownedPtr( new OBVSpatialMedianSplitter<OBV>() );
		}


        /**
         * @brief creates a splitter strategy for obb tree construction using mean
         * splitting strategy.
         *
         * The strategy splits a trimesh in one of the axes of the OBB. The splitting
         * point is determined as the mean on one of the OBB axes. Performs in O(n)
         * @return Splitter strategy for OBB tree construction
         */
		template<class OBV>
		rw::common::Ptr<BVTreeFactory::BVSplitterStrategy<OBV> > makeOBVMeanSplitter(){
		    return rw::common::ownedPtr( new OBVMeanSplitter<OBV>() );
		}


        /**
         * @brief creates an OBB factory that fits obb's to triangle meshes
         * by calculating the covariance of the triangle mesh and use the eigen vectors
         * of the covariance as the axes of the OBB.
         * @return OBB factory
         */
		template<class T>
		rw::common::Ptr<geometry::BVFactory<rw::geometry::OBB<T> > > makeOBBCovarFactory(){
		    return rw::common::ownedPtr( new geometry::OBBFactory<T>(geometry::OBBFactory<T>::PCA) );
		}

		/**
		 * @brief creates an OBB tree using a covariance OBB factory and a object median splitting strategy.
		 * @param mesh [in] the mesh that should be decomposed into a OBB tree
		 * @param maxTrisInLeaf [in] the maximum number of tris that are allowed in each leaf node
		 * @return OBB tree
		 */
		template<class BVTREE>
		BVTREE* makeTopDownOBBTreeCovarMedian(rw::geometry::TriMesh::Ptr mesh, int maxTrisInLeaf=1){
		    typedef typename Traits<BVTREE>::BVType BVType;
			rw::common::Ptr<geometry::BVFactory<typename Traits<BVTREE>::BVType > > bvfactory = makeOBBCovarFactory<typename Traits<BVType>::value_type>();
			rw::common::Ptr<BVSplitterStrategy<typename Traits<BVTREE>::BVType > > splitter = makeOBVMedianSplitter<BVType>();
			return makeTopDownTree<BVTREE>(mesh, *bvfactory, *splitter, maxTrisInLeaf);
		}

		/**
		 * @brief Creates an OBB tree using a object median splitting strategy.
		 * @param shell [in] the shell to create OBB tree for.
		 * @param maxTrisInLeaf [in] the maximum number of faces in a leaf node.
		 * @return the OBB tree.
		 */
		template<class BVTREE>
		BVTREE* makeTopDownOBBTreeMedian(rw::geometry::Shell::CPtr shell, int maxTrisInLeaf = 1)
		{
		        using rw::geometry::BVFactory;
		        typedef typename Traits<BVTREE>::BVType BVType;
		        rw::common::Ptr<BVFactory<typename Traits<BVTREE>::BVType > > bvfactory =
		                makeOBBCovarFactory<typename Traits<BVType>::value_type>();
		        rw::common::Ptr<BVShellSplitterStrategy<typename Traits<BVTREE>::BVType > > splitter =
		                makeOBVShellMedianSplitter<BVType>();
		        return makeTopDownTree<BVTREE>(shell, *bvfactory, *splitter, maxTrisInLeaf);
		}

		template<class BVTREE>
		BVTREE* makeTopDownOBBTreeCovarSpatialMedian(rw::geometry::TriMesh::Ptr mesh, int maxTrisInLeaf=1){
		    typedef typename Traits<BVTREE>::BVType BVType;
		    rw::common::Ptr<geometry::BVFactory<typename Traits<BVTREE>::BVType > > bvfactory = makeOBBCovarFactory<typename Traits<BVType>::value_type>();
            rw::common::Ptr<BVSplitterStrategy<typename Traits<BVTREE>::BVType > > splitter = makeOBVSpatialMedianSplitter<BVType>();
            return makeTopDownTree<BVTREE>(mesh, *bvfactory, *splitter, maxTrisInLeaf);
        }

		template<class BVTREE>
		BVTREE* makeTopDownOBBTreeCovarMean(rw::geometry::TriMesh::Ptr mesh, int maxTrisInLeaf=1){
		    typedef typename Traits<BVTREE>::BVType BVType;
			rw::common::Ptr<geometry::BVFactory<BVType> > bvfactory = makeOBBCovarFactory<typename Traits<BVType>::value_type>();
			rw::common::Ptr<BVSplitterStrategy<BVType> > splitter = makeOBVMeanSplitter<BVType>();
			return makeTopDownTree<BVTREE>(mesh, *bvfactory, *splitter, maxTrisInLeaf);
		}

		template<class BVTREE>
		BVTREE* makeTopDownOBBIdxTreeCovarMedian(rw::geometry::TriMesh::Ptr mesh, int maxTrisInLeaf=1){
		    typedef typename Traits<BVTREE>::BVType BVType;
            rw::common::Ptr<geometry::BVFactory<typename Traits<BVTREE>::BVType > > bvfactory = makeOBBCovarFactory<typename Traits<BVType>::value_type>();
            rw::common::Ptr<BVSplitterStrategy<typename Traits<BVTREE>::BVType > > splitter = makeOBVMedianSplitter<BVType>();
            return makeTopDownTree<BVTREE>(mesh, *bvfactory, *splitter, maxTrisInLeaf);
        }




		/**
		 * @brief general function for constructing a binary bounding volume tree in a top down fashion.
		 *
		 * @param mesh [in] the mesh on which to construct the bounding volume tree
		 * @param bvFactory [in] a factory for creating/fitting bounding volumes given a triangle mesh
		 * @param splitter [in] divides a mesh into 2 meshes by sorting the mesh and providing a splitting index
		 * @param maxTrisInLeaf [in] the maximum number of tris that are allowed in each leaf node
		 * @return
		 */
		template<class BVTREE>
		static BVTREE* makeTopDownTree(rw::geometry::TriMesh::Ptr mesh,
		                               geometry::BVFactory<typename Traits<BVTREE>::BVType>& bvFactory,
		                               const BVSplitterStrategy<typename Traits<BVTREE>::BVType>& splitter,
		                               int maxTrisInLeaf=1)
		{
		    typedef typename Traits<BVTREE>::BVType BVType;
		    typedef typename Traits<BVType>::value_type value_type;
		    typedef typename Traits<BVTREE>::NodeIterator NodeIterator;
			using namespace rw::math;
			using namespace rw::geometry;
			using namespace rw::common;

			// we create the binary tree
			BVTREE* tree = new BVTREE( new TriMeshAccessor<value_type>(mesh) );

			// create a proxy for the triangle mesh
			IndexedTriArray<> idxArray(mesh);
			// now for each tri soup indicated by the triangle indexes compute a OBB sub tree
			//Timer t;
			NodeIterator root = tree->createRoot();
			//std::cout << "recursiveTopDownTree: " << std::flush;
			recursiveTopDownTree<BVTREE>(tree, root, idxArray, bvFactory, splitter, maxTrisInLeaf);
			//std::cout << t.toString() << std::endl;
			//std::cout << "IDX MAP ARRAY" << std::endl;
			//BOOST_FOREACH(int idx, idxArray.getIndexes()){
			//    std::cout << idx << "\n";
			//}

			//std::cout << "tree prims: " << (*root)->nrOfPrims() << std::endl;

			return tree;
		}

		/**
		 * @brief Top-down construction of a bounding volume tree.
		 * @param shell [in] the shell to create bounding volume tree for.
		 * Must have at least one face.
		 * @param bvFactory a factory for creating bounding volumes.
		 * @param splitter [in] the splitting strategy used to divide the
		 * shell.
		 * @param maxTrisInLeaf [in] how many faces to allow in one leaf of the
		 * tree.
		 * @return the resulting bounding volume tree.
		 */
		template<class BVTREE>
		static BVTREE* makeTopDownTree(rw::geometry::Shell::CPtr shell,
		        geometry::BVFactory<typename Traits<BVTREE>::BVType>& bvFactory,
		        const BVShellSplitterStrategy<typename Traits<BVTREE>::BVType>& splitter,
		        int maxTrisInLeaf=1)
		{
		        typedef typename Traits<BVTREE>::BVType BVType;
		        typedef typename Traits<BVType>::value_type value_type;
		        typedef typename Traits<BVTREE>::NodeIterator NodeIterator;
		        using rw::geometry::IndexedFaceArray;

		        // we create the binary tree
		        BVTREE* tree = new BVTREE(new ShellAccessor<value_type>(shell));
		        IndexedFaceArray idxArray(shell);
		        NodeIterator root = tree->createRoot();
		        recursiveTopDownTree<BVTREE>(tree, root, idxArray, bvFactory,
		                splitter, maxTrisInLeaf);

		        return tree;
		}

		/**
		 * @brief recursive top down construction of a bounding volume tree
		 * @param tree
		 * @param node
		 * @param mesh
		 * @param bvFactory
		 * @param splitter
		 * @param maxTrisInLeaf
		 */
		template<class BVTREE>
		static void recursiveTopDownTree(BVTree<BVTREE>* tree,
									typename Traits<BVTREE>::NodeIterator &node,
									rw::geometry::IndexedTriArray<> mesh,
									rw::geometry::BVFactory<typename Traits<BVTREE>::BVType>& bvFactory,
									const BVSplitterStrategy<typename Traits<BVTREE>::BVType> &splitter,
									size_t maxTrisInLeaf)
		{
            typedef typename Traits<BVTREE>::BVType BVType;
            //typedef typename Traits<BVType>::value_type value_type;
            typedef typename Traits<BVTREE>::NodeIterator NodeIterator;

			if(mesh.getSize()==0){
			    RW_ASSERT(0); // we should not arrive at this.
			} else if(mesh.getSize()<=maxTrisInLeaf){
				// make a leaf node
			    BVType bv = bvFactory.makeBV( mesh );
			    tree->setPrimIdx( (int) mesh.getGlobalIndex(0), node);
			    tree->setNrOfPrims( (int) mesh.getSize(), node);
			    tree->setBV(bv, node);
			} else {
				// create a bounding volume of the mesh and split it

				BVType bv = bvFactory.makeBV( mesh );
				tree->setBV( bv , node);
				//std::cout << "Range: "<< mesh.getGlobalIndex(0) << ";" << mesh.getGlobalIndex(mesh.getSize()) << std::endl;
				// were to split the mesh (the order in the mesh might be changed in this function)
                //rw::common::Timer t;
				size_t k = splitter.partitionMesh(mesh, bv );
				//std::cout << "split, " << mesh.getSize()<<" : " << t.toString("ss:zz") << "\n";
				// left child

				if(k>0){
                    NodeIterator leftnode = tree->createLeft( node );
                    recursiveTopDownTree(tree, leftnode, mesh.getSubRange(0,k), bvFactory, splitter, maxTrisInLeaf);
				}

				// right child
				if(k<mesh.getSize()){
				    NodeIterator rightnode = tree->createRight( node );
                    recursiveTopDownTree(tree, rightnode, mesh.getSubRange(k,mesh.getSize()), bvFactory, splitter, maxTrisInLeaf);
				}
			}
		}

		/**
		 * @brief Recursive top-down construction of a bounding volume tree.
		 *
		 * @param tree [out] the resulting bounding volume tree.
		 * @param node [in/out] current node being handled in tree.
		 * @param shell [in] the shell to create bounding volume tree for.
		 * Must have at least one face.
		 * @param bvFactory a factory for creating bounding volumes.
		 * @param splitter [in] the splitting strategy used to divide the
		 * shell.
		 * @param maxTrisInLeaf [in] how many faces to allow in one leaf of the
		 * tree.
		 */
		template<class BVTREE>
		static void recursiveTopDownTree(BVTree<BVTREE>* tree,
		        typename Traits<BVTREE>::NodeIterator &node,
		        rw::geometry::IndexedFaceArray shell,
		        rw::geometry::BVFactory<typename Traits<BVTREE>::BVType>& bvFactory,
		        const BVShellSplitterStrategy<typename Traits<BVTREE>::BVType> &splitter,
		        const size_t maxTrisInLeaf)
		{
		        typedef typename Traits<BVTREE>::BVType BVType;
		        typedef typename Traits<BVTREE>::NodeIterator NodeIterator;

		        if(shell.size() == 0) {
		            RW_ASSERT(0); // we should not arrive at this.
		        } else if(shell.size()<=maxTrisInLeaf){
		            // make a leaf node
		            BVType bv = bvFactory.makeBV( shell );
		            tree->setPrimIdx( (int) shell.getGlobalIndex(0), node);
		            tree->setNrOfPrims( (int) shell.size(), node);
		            tree->setBV(bv, node);
		        } else {
		            // create a bounding volume of the mesh and split it

		            BVType bv = bvFactory.makeBV( shell );
		            tree->setBV( bv , node);
		            // were to split the mesh (the order in the mesh might be changed in this function)
		            //rw::common::Timer t;
		            const std::size_t k = splitter.partitionShell(shell, bv);
		            // left child

		            if(k > 0) {
		                rw::geometry::IndexedFaceArray arr = shell.getSubRange(0,k);
		                NodeIterator leftnode = tree->createLeft( node );
		                recursiveTopDownTree(tree, leftnode, arr, bvFactory, splitter, maxTrisInLeaf);
		            }

		            // right child
		            if(k < shell.size()) {
		                rw::geometry::IndexedFaceArray arr = shell.getSubRange(k,shell.size());
		                NodeIterator rightnode = tree->createRight( node );
		                recursiveTopDownTree(tree, rightnode, arr, bvFactory, splitter, maxTrisInLeaf);
		            }
		        }
		}


	    /**
	     * @brief computes a score of a specific splitting axis. Basically each triangle in the mesh is
	     * visited and all vertices are tested for being on either left or right side of the splitting
	     * value. The score is then the total number of triangles being completely on either left or right
	     * side. Which means that a score=mesh.size() is the best score where the split value completely splits
	     * the triangles into two volumes. score=0 is the worst where all triangles has vertices on both sides on the split value.
	     * @param mesh
	     * @param splitValue
	     * @param t3d
	     * @return
	     */
	    template<class T>
	    static int evaluateSplitAxis(rw::geometry::IndexedTriArray<>& mesh, int splitAxis, double splitValue, const rw::math::Transform3D<T>& t3d){
	        using namespace rw::geometry;

	        int left=0, right=0;
	        Triangle<T> tri;
	        for(size_t i=0; i<mesh.getSize(); i++ ){
	            // transform the vertex point to the obb root
	            mesh.getTriangle(i, tri);
	            bool toLeft = (t3d*tri.getVertex(0))[splitAxis] < splitValue;
	            if(toLeft){
	                // check if its really to the left
	                toLeft &= (t3d*tri.getVertex(1))[splitAxis] < splitValue;
	                toLeft &= (t3d*tri.getVertex(2))[splitAxis] < splitValue;
	                if(toLeft==true) left++;
	            } else {
	                toLeft |= (t3d*tri.getVertex(1))[splitAxis] < splitValue;
	                toLeft |= (t3d*tri.getVertex(2))[splitAxis] < splitValue;
	                if(toLeft==false) right++;
	            }
	        }
	        return right+left;
	    }

	    /**
	     * @brief Evaluate the quality of a split axis.
	     *
	     * A good split axis divides the object such that the number of faces
	     * lying completely to the left or right of the axis is maximized.
	     *
	     * @note Remember to call sortAxis() on the \b shell for the given axis
	     * before calling this method.
	     *
	     * @param shell [in] the shell where each IndexedFace contains the
	     * face extremums along the given axis.
	     * @param splitValue [in] the split axis value.
	     * @return the number of faces lying completely to the left or right of
	     * the split axis.
	     */
	    static int evaluateSplitAxis(const rw::geometry::IndexedFaceArray& shell, const double splitValue)
	    {
	        int left=0, right=0;
	        rw::geometry::IndexedFaceArray::IndexedFace face;
	        for(size_t i=0; i < shell.size(); i++) {
	            // transform the vertex point to the obb root
	            shell.getIndexedFace(i, face);
	            bool toLeft = face.lower < splitValue;
	            if(toLeft){
	                // check if its really to the left
	                toLeft &= face.upper < splitValue;
	                if(toLeft) left++;
	            } else {
	                toLeft |= face.upper < splitValue;
	                if(!toLeft) right++;
	            }
	        }
	        return right+left;
	    }

	    /**
	     * @brief Object median splitting strategy using axis of largest variance. Splits the mesh
	     * in the median on the axis with largest variance.
	     */
	    template<class BV>
	    struct OBVMedianSplitter: public BVTreeFactory::BVSplitterStrategy<BV> {
	    public:
	        size_t partitionMesh(rw::geometry::IndexedTriArray<>& mesh, const BV& obb) const {
	            using namespace rw::geometry;
	            using namespace rw::math;
	            using namespace rw::common;
	            Transform3D<> t3d = inverse( cast<double>(obb.getTransform()) );

	            int splitAxis = 0, bestSplitAxis = 0; // choose longest (x-axis) for splitting the Box
	            int bestSplitScore =0;
	            int median = (int)(mesh.getSize()/2);
	            //Timer time;
	            do{
	                // calculate median
	                // sort all indexes in trisIdx
	                rw::common::Timer t;
	                mesh.sortAxis(splitAxis, t3d );
	                //std::cout << "sort   " << t.toString("ss:zz") << "\n";
	                // now make sure that the choosen split axis is not a bad one
	                Triangle<> tri = mesh.getTriangle(median);
	                Vector3D<> center = t3d*((tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0);
	                //t.reset();
	                int score = evaluateSplitAxis(mesh, splitAxis, center[splitAxis], t3d);
                    //std::cout << "eval   " << t.toString("ss:zz") << "\n";
	                if(score>bestSplitScore){
	                    bestSplitScore = score;
	                    bestSplitAxis = splitAxis;
	                }

	                // criteria for an okay splitting point
	                if( score + mesh.getSize()/8 >= mesh.getSize() ){
	                    break;
	                }

	                splitAxis++;
	                if(splitAxis==3){
	                    // No axis was goood, so we use the best one
	                    if(bestSplitAxis!=2)
	                        mesh.sortAxis(bestSplitAxis, t3d);
	                    break;
	                }

	            } while( splitAxis<3 );

	            return median;
	        }
	    };

        /**
         * @brief Spatial Median splitting strategy for shell of analytical
         * surfaces.
         *
         * Each of the three axes of the shell OBB is tested as good splitting
         * axes. The first axis that causes 87.5\% of the faces to lie
         * completely to the left or right of the split axis is used directly.
         * If this is not the case for any axis, the best axis is chosen.
         */
	    template<class BV>
	    struct OBVShellMedianSplitter: public BVTreeFactory::BVShellSplitterStrategy<BV> {
	        public:
	            //! @copydoc BVShellSplitterStrategy::partitionShell
	            size_t partitionShell(rw::geometry::IndexedFaceArray& shell, const BV& obb) const
	            {
	                using rw::geometry::IndexedFaceArray;
	                using namespace rw::math;
	                Transform3D<> t3d = inverse( cast<double>(obb.getTransform()) );

	                int splitAxis = 0, bestSplitAxis = 0; // choose longest (x-axis) for splitting the Box
	                int bestSplitScore = 0;
	                const int median = (int)(shell.size()/2);
	                do{
	                    // calculate median
	                    // sort all indexes in trisIdx
	                    shell.sortAxis(splitAxis, t3d );
	                    // now make sure that the choosen split axis is not a bad one
	                    const IndexedFaceArray::IndexedFace& face = shell.getIndexedFace(median);
	                    const int score = evaluateSplitAxis(shell, face.center);
	                    if(score > bestSplitScore) {
	                        bestSplitScore = score;
	                        bestSplitAxis = splitAxis;
	                    }

	                    // criteria for an okay splitting point
	                    if( score + shell.size()/8 >= shell.size() ){
	                        break;
	                    }

	                    splitAxis++;
	                    if(splitAxis==3){
	                        // No axis was goood, so we use the best one
	                        if(bestSplitAxis!=2)
	                            shell.sortAxis(bestSplitAxis, t3d);
	                        break;
	                    }

	                } while( splitAxis<3 );

	                return median;
	            }
	    };

	    /**
	     * @brief Spatial Median splitting strategy. The median of the bounding volume projection
	     * extends are used as splitting point.
	     */
	    template<class BV>
	    class OBVMeanSplitter: public BVTreeFactory::BVSplitterStrategy<BV> {
	    public:
	        //BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<rw::geometry::OBV<BV>, BV>::value) , "Bounding volume MUST inherit geometry::OBV");

	        size_t partitionMesh(rw::geometry::IndexedTriArray<>& mesh, const BV& obb) const {

                using namespace rw::geometry;
                using namespace rw::math;
                using namespace rw::common;
                RW_ASSERT(mesh.getSize()>0);

	            Transform3D<> t3d = inverse( cast<double>(obb.getTransform()) );

	            int splitAxis = 0, bestSplitAxis = 0; // choose longest (x-axis) for splitting the Box
	            int bestSplitScore =0;
	            double mean = 0;
	            do{
	                RW_ASSERT(splitAxis>=0);
	                RW_ASSERT(splitAxis<3);
	                // We need to calculate the mean for a splitting axis
	                mean=0;
	                for(size_t i=0; i<mesh.getSize(); i++ ){
	                    // transform the vertex point to the obb root
	                    Triangle<> tri = mesh.getTriangle(i);
	                    Vector3D<> center = t3d*((tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0);
	                    mean += center[splitAxis];
	                }
	                mean = mean/mesh.getSize();

	                int score = evaluateSplitAxis(mesh, splitAxis, mean, t3d);
	                if(score>bestSplitScore){
	                    bestSplitScore = score;
	                    bestSplitAxis = splitAxis;
	                }

	                // criteria for an okay splitting point
	                if( score + mesh.getSize()/8 >= mesh.getSize() ){
	                    break;
	                }

                    splitAxis++;

	            } while( splitAxis<3 );

	            unsigned int meanSplit = 0;
	            double mindist = DBL_MAX;
	            // find the mean split
	            // now we sort the best split axis
	            mesh.sortAxis(bestSplitAxis, t3d);
	            // and find the splitting index
	            for(size_t i=0; i<mesh.getSize(); i++ ){
	                // transform the vertex point to the obb root
	                Triangle<> tri = mesh.getTriangle(i);
	                Vector3D<> center = t3d*((tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0);
	                if( fabs(center[bestSplitAxis]-mean)<mindist ){
	                    mindist = fabs(center[bestSplitAxis]);
	                    meanSplit = i;
	                }
	            }

	            if(meanSplit==0)
	                return 1;
	            if(meanSplit==mesh.size()-1)
	                return meanSplit-1;
	            return meanSplit;
	        }
	    };

	    /**
	     * @brief Spatial Median splitting strategy. The median of the bounding volume projection
	     * extends are used as splitting point.
	     */
	    template<class BV>
	    struct OBVSpatialMedianSplitter: public BVTreeFactory::BVSplitterStrategy<BV> {
	    public:
	        BOOST_STATIC_ASSERT( (boost::is_base_of<rw::geometry::OBV<BV>, BV>::value) );

	        size_t partitionMesh(rw::geometry::IndexedTriArray<>& mesh, const BV& obb) const {
                using namespace rw::geometry;
                using namespace rw::math;
                using namespace rw::common;

	            //std::cout << "partition mesh: " << mesh.size() << std::endl;
	            Transform3D<> t3d = inverse( cast<double>(obb.getTransform()) );

	            size_t splitAxis = 0, bestSplitAxis = 0; // choose longest (x-axis) for splitting the Box
	            size_t bestSplitScore = 0;

	            do{
	                unsigned int score = evaluateSplitAxis(mesh, splitAxis, 0, t3d);
	                if(score>bestSplitScore){
	                    bestSplitScore = score;
	                    bestSplitAxis = splitAxis;
	                }

	                // criteria for an okay splitting point
	                if( score + mesh.getSize()/8 >= mesh.getSize() ){
	                    break;
	                } else {
	                    splitAxis++;
	                }

	            } while( splitAxis<3 );
	            //std::cout << "Split axis: " << bestSplitAxis << std::endl;
	            double mindist = DBL_MAX;
	            unsigned int closest = 0;
	            // now we sort the best split axis
	            mesh.sortAxis(bestSplitAxis, t3d);
	            // and find the splitting index
	            for(size_t i=0; i<mesh.getSize(); i++ ){
	                // transform the vertex point to the obb root
	                Triangle<> tri = mesh.getTriangle(i);
	                Vector3D<> center = t3d*((tri.getVertex(0)+tri.getVertex(1)+tri.getVertex(2))/3.0);
	                if( fabs(center[bestSplitAxis])<mindist ){
	                    mindist = fabs(center[bestSplitAxis]);
	                    closest = i;
	                }
	            }
	            //std::cout << "closest: " << closest << std::endl;
	            if(closest==0)
	                return 1;
	            if(closest==mesh.size()-1)
	                return closest-1;
	            return closest;
	        }
	    };



	};


}
}

#endif /*OBBTREE_HPP_*/
