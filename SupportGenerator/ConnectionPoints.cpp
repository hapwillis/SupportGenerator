#include "ConnectionPoints.h"



ConnectionPoints::ConnectionPoints()
{


	//var numSeeds = 5; // Must be >> number of disconnected support areas
	//var centerpoint = [];
	//var processing = [];

	// IDEA: Option to start at model vertices (ie lowest point)
	// Llyod's algorithm gives slightly better results at a much lower speed
	// If overhangs are split, generate one seed point per area

	//var triIndex = 9 * Math.floor(triList.length*Math.random() / 9);
	//var winningP = randPoint(triIndex); // Random point from random triangle
	//octree.add(winningP); // Add point to candidates octree
	//supports.push(winningP); // Add point to list of supports
	//processing.push(winningP); // Add point to open list
	//connectorCandidates.push(triList.slice(triIndex, (triIndex + 9)));

	//for (var i = 0; i < numSeeds; i++) {
	//	for (var k = 0; k < numSeeds; k++) {
	//		triIndex = 9 * Math.floor(triList.length*Math.random() / 9);
	//		winningP = randPoint(triIndex); // Get random point + add to open list
	//		var distance = winningP.distanceTo(octree.findClosest(winningP));
	//		if (distance > minD) { // Discard if too close to any other point
	//			octree.add(winningP);
	//			supports.push(winningP);
	//			processing.push(winningP);
	//			connectorCandidates.push(triList.slice(triIndex, (triIndex + 9)));
	//			k = numSeeds; // Exit loop if point is valid
	//		}
	//	}
	//	while (processing.length > 0) { // While processing[] has points
	//		console.log("Processing length: " + processing.length);
	//		var p = processing.pop(); // Pop point from processing[]
	//		var subset = triOctree.findInRadius(p, radius, minD); // FIXME: This is broken
	//		for (var k = 0; k < numSeeds; k++) {
	//			// Note: triIndex not weighted by area (ie big tris are underfilled)
	//			triIndex = Math.floor(subset.length*Math.random());
	//			winningP = randPoint(subset[triIndex]);
	//			// IDEA: Scale with slope, make sure point is within annulus
	//			var distance = winningP.distanceTo(octree.findClosest(p));
	//			if ((distance > minD) && (distance < radius)) {
	//				// Problem: returned point can be > radius fairly easily
	//				octree.add(winningP);
	//				supports.push(winningP);
	//				processing.push(winningP);
	//				connectorCandidates.push(triList.slice(triIndex, (triIndex + 9)));
	//			}
	//		}
	//	}
	//	// Optionally if the generated mass is too small, discard these points
	//}

}


ConnectionPoints::~ConnectionPoints()
{
}

glm::vec3 ConnectionPoints::randomPoint(Face * face)
{
	//var r1 = Math.random();
	//var r2 = (1 - r1) * Math.random();
	//var r3 = (1 - r1 - r2);

	//var x = r1 * triList[t] + r2 * triList[t + 3] + r3 * triList[t + 6];
	//var y = r1 * triList[t + 1] + r2 * triList[t + 4] + r3 * triList[t + 7];
	//var z = r1 * triList[t + 2] + r2 * triList[t + 5] + r3 * triList[t + 8];

	return glm::vec3();
}
