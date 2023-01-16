#ifndef COLCAL_H
#define COLCAL_H

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
// GEOMETRY SUPPORT
// * POINT
// * TRIANGLE
// * PLANE
// * BOX
// * SHPERE
// * RAY
// * SEGEMENT
// 
// BOUNDING VOLUME
// * BOX
// * SPHERE
// 
// ACCELERATION COLLISION DETECTION ALGORITHM
// * SWEEP AND PRUNE (SaP)
// * BOUNDING VOLUME HIERARCHY (BVH)
// * OCTREE (OCT)
//
// In this library, I implemented some collision detection methods, you 
// can use this library for geometrical or physical simultaion.
// 
//
// NOTE: this lib uses right-hand rule	
// 
//	   |y
//	   |   / z
//	   |  /
//	   | /
//	   |/_ _ _ _ _ x
// 
/////////////////////////////////////////////////////////////////

#include <vector>
#define ColCal_Max(a,b) ((a) > (b) ? (a) : (b))
#define ColCal_Min(a,b) ((a) < (b) ? (a) : (b))

#endif // !COLCAL_H
