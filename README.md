predicts the final resting positions of two billiard balls, accounting for friction and one potential elastic collision

the logic is broken into three main parts:

1  **motion under friction**
   a ball with initial speed `v` will travel a total distance of `v / 0 8` before stopping this simple rule governs all movement

2  **collision detection**
   the simulation uses a distance-based geometric check it calculates the closest point between the moving ball's trajectory and the static ball's center if this distance is less than two radii, a collision is possible it then confirms the ball can travel the required distance to the contact point before friction stops it

3  **collision resolution**
   if a collision occurs, it's treated as perfectly elastic the velocity of the incoming ball is decomposed into components parallel and perpendicular to the line connecting the centers the parallel components are exchanged, and the perpendicular components are kept
