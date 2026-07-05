# Context

### Bottom camera
The downward-facing camera used by the diansai_first mission to observe the ground below the aircraft.

### Mission height
The altitude value used by diansai_first for flight control and reach checks. It comes from the filtered height received over the STM32 serial link, not from the laser-array ground estimator.

### Original ground camera model
The camera model previously used as the ground-facing camera and now required for the diansai_first bottom camera.


### diansai_first mission
A three-point competition mission in which the aircraft takes off to a takeoff point, flies to a pickup point to collect a ground iron block using the arm and magnet, transports it to an endpoint, releases the block there, retracts the arm, lands, and completes the task.

### Takeoff point
The first mission point after start: a safe hover pose above the start area before traveling to the pickup point.

### Pickup point
The mission point above the ground iron block where the aircraft descends to a pickup hover height, then performs the pickup action sequence with the arm and magnet.

### Pickup hover height
The low hover height at the pickup point: high enough to avoid landing, low enough that the deployed arm can touch the ground iron block.

### Pickup action sequence
At the pickup point, the aircraft first descends to pickup hover height, deploys the arm, enables the magnet to attach the iron block, then retracts the arm before transporting the block.

### Transport cruise point
The mission point used to carry the iron block toward the destination at a safe travel height.

### Endpoint
The final mission point where the aircraft releases the iron block, retracts the arm, lands, and completes the mission.

### Dropoff hover height
The low hover height at the endpoint: high enough to avoid landing before release, low enough that the deployed arm can place the iron block near the ground.

### Dropoff action sequence
At the endpoint, the aircraft descends to dropoff hover height, deploys the arm, disables the magnet to release the iron block, retracts the arm, then lands and completes the mission.
