---
layout: page
title: Frequently Asked Questions (FAQ)
category: Documentation
---

This page discusses several questions that are often asked by our
users.

### When creating a Model from an urdf file, i cannot access any frames

- The frames are a way to keep track of special points (such as grippers etc..) in the kinematic chain. You can add a
  frame attached to a parent joint in the Model by hand but if you don't specify it yourself, no frames will be created
  automatically except when anchor/fixed joints are encountered in the urdf file. If it is the case, a frame is added
  at the center of the fixed joint and the next body inertial informations are merged to its parent.
