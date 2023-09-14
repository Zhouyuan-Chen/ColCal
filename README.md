```
/////////////////////////////////////////////////////////////////
///////       COLLISION CALCULATION LIBRARY (ColCal)      ///////
///////  _____   _____   _       _____       ___   _      ///////
/////// / ___|  /  _  \ | |     /  ___|     /   | | |     ///////
/////// | |     | | | | | |     | |        / /| | | |     ///////
/////// | |     | | | | | |     | |       / / | | | |     ///////
/////// | |___  | |_| | | |___  | |___   / /  | | | |___  ///////
/////// \_____| \_____/ |_____| \_____| /_/   |_| |_____| ///////
///////                                                   ///////
/////////////////////////////////////////////////////////////////
// @author Zhouyuan Chen
// @date 2023/1/16
/////////////////////////////////////////////////////////////////
// DESCRIPTION
/////////////////////////////////////////////////////////////////
// In this library, I implemented some collision detection methods,
// you can use this library for geometrical or physical simulation.
// If you encounter any problems while you using this library, please 
// do not hesitate to contact me through Git Hub!
/////////////////////////////////////////////////////////////////
// Dependency
/////////////////////////////////////////////////////////////////
// * Eigen for Eigenvectors solution
//  you can download this lib in following website link
// https://eigen.tuxfamily.org/index.php?title=Main_Page
/////////////////////////////////////////////////////////////////
// GEOMETRY SUPPORT
/////////////////////////////////////////////////////////////////
// * POINT(implemented)
// * TRIANGLE(implemented)
// * PLANE(implemented)
// * BOX(implemented)
/////////////////////////////////////////////////////////////////
// BOUNDING VOLUME SUPPORT
/////////////////////////////////////////////////////////////////
// * AABB-BOX(implemented)
// * OBB-BOX(implemented)
// * BoundingBox Adapter(implemented)(Due to design reasons,
//    I don't recommend you to use the boundingbox class to use 
//    the polymorphic features of the class.)
/////////////////////////////////////////////////////////////////
// ACCELERATION COLLISION DETECTION ALGORITHM SUPPORT
/////////////////////////////////////////////////////////////////
// Note: I only used AABB now, OBB will be realized soon.
// * SWEEP AND PRUNE (SaP)(implemented)
// * KDTREE (KDT + KNN-Search)(implemented)
// * BOUNDING VOLUME HIERARCHY (BVH SAH + FPQ)(implemented)
// * * following features would be implemented in the future
// * QUTREE (QUT)
// * OCTREE (OCT)
// * HASHING SPACE PARTITION (HSP)
/////////////////////////////////////////////////////////////////
// MATH CLASS SUPPORT
/////////////////////////////////////////////////////////////////
// * VEC3 (for normal vector)(implemented)
// * VEC4 (for point position)(implemented)
// * MAT4 (for basic rotate and transform)(implemented)
// * Eigenvectors Solution (Using Eigen Library)
/////////////////////////////////////////////////////////////////
// DIAGRAM FOR LEFT-HAND RULE
/////////////////////////////////////////////////////////////////
//	
//     |y
//     |   / z
//     |  /
//     | /
//     |/_ _ _ _ _ x
// 
// NOTE: this lib uses LEFT-hand rule
/////////////////////////////////////////////////////////////////
// DIAGRAM FOR BOX COLLISION
/////////////////////////////////////////////////////////////////
//      	 
//         _ _ _ _ _ _		   
//        /           /|<---behind
//       /    top    / |
//      /_ _ _ _ _ _/  | 
//left->|           |<-|- - - - right
//      |           |  |
//      |   front   |  / 
//      |           | /
//      |_ _ _ _ _ _|/
//          ^
//          |
//        bottom
// 
//     |y
//     |   / z
//     |  /
//     | /
//     |/_ _ _ _ _ x
// 
/////////////////////////////////////////////////////////////////
// DIAGRAM FOR PLANE
/////////////////////////////////////////////////////////////////
// 
//             ^  arrow 
//         _ _ | _ _ _ _  
//        /    |        /
//       /     * point /
//      /             /
//     /_ _ _ _ _ _ _/
// 
/////////////////////////////////////////////////////////////////
// DIAGRAM FOR TRIANGLE
/////////////////////////////////////////////////////////////////
//      
//            * v3
//           / \
//          /   \
//         /     \
//        /       \
//       /         \
//  v1  *- - - - - -* v2
// 
// the order of vertieces' idex: v1->v2->v3
// 
/////////////////////////////////////////////////////////////////
```
