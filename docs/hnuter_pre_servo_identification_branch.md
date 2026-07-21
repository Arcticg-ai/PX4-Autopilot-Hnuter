# Hnuter pre-servo-identification snapshot

This branch preserves the complete Hnuter controller and allocator immediately
before identified servo dynamics are applied to the plant. Gazebo servo topics
drive the four joint position controllers directly, tilt transmission gains are
unity, and the optional allocator-side tilt dynamics estimator is disabled.

The full primary and secondary mechanical travel remains enabled; this branch
therefore retains the later control and safety fixes without including the
identified gain, delay, first-order lag, or directional rate model.
