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

In this library, I implemented some collision detection methods, you can use this library for geometrical or physical simultaion. If this library has any problem, please request the issue to let me know. Hope this can help you build your own work!

### DESCRIPTION
 * GEOMETRY SUPPORT
 * * POINT
 * * TRIANGLE
 * * PLANE
 * * BOX
 * * SHPERE
 * * RAY
 * * SEGEMENT
 
 * BOUNDING VOLUME SUPPORT
 * * BOX
 * * SPHERE
 
 * ACCELERATION COLLISION DETECTION ALGORITHM
 * * SWEEP AND PRUNE (SaP)
 * * BOUNDING VOLUME HIERARCHY (BVH)
 * * OCTREE (OCT)

 * NOTE: this lib uses right-hand rule	
 
	   |y
	   |   / z
	   |  /
	   | /
	   |/_ _ _ _ _ x
  
 * NOTE: digram for box	
		      _ _ _ _ _ _		   
		     /			     /|<---behind
		    /	top		    / |
		   /_ _ _ _ _ _/  | 
 left->|			     |<-|- - - - right
		   |			     |  |
		   |	front	   |  / 
		   |			     | /
		   |_ _ _ _ _ _|/
			     ^
			     |
        bottom
