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
// GEOMETRY SUPPORT
/////////////////////////////////////////////////////////////////
// * POINT
// * TRIANGLE
// * PLANE
// * BOX
// * SHPERE
// * RAY
// * SEGEMENT
/////////////////////////////////////////////////////////////////
// BOUNDING VOLUME SUPPORT
/////////////////////////////////////////////////////////////////
// * BOX (AABB-BOX)
// * SPHERE
/////////////////////////////////////////////////////////////////
// ACCELERATION COLLISION DETECTION ALGORITHM SUPPORT
/////////////////////////////////////////////////////////////////
// * SWEEP AND PRUNE (SaP)
// * QUTREE (QUT)
// * OCTREE (OCT)
// * KDTREE (KDT)
// * BOUNDING VOLUME HIERARCHY (BVH)
/////////////////////////////////////////////////////////////////
// MATH CLASS SUPPORT
/////////////////////////////////////////////////////////////////
// * VEC3 (for normal vector)
// * VEC4 (for point position)
// * MAT4 (for basic rotate and transform)
/////////////////////////////////////////////////////////////////
// DIAGRAM FOR LEFT-HAND RULE
/////////////////////////////////////////////////////////////////
//	
//	   |y
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
//	   |   / z
//	   |  /
//	   | /
//	   |/_ _ _ _ _ x
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


#ifndef COLCAL_H
#define COLCAL_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>

#define ColCal_Max(a,b) ((a) > (b) ? (a) : (b))
#define ColCal_Min(a,b) ((a) < (b) ? (a) : (b))

typedef float ColCal_DataType;
//typedef double ColCal_DataType;
#define ColCal_Pi 3.1415926
//#define ColCal_PROXIMATE_ONE 0.999999999999
//#define ColCal_PROXIMATE_ZERO 0.00001

#endif // !COLCAL_H
