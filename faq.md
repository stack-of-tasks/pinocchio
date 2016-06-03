---
layout: default
title: Frequently Asked Questions (FAQ)
group: "navigation"
disqus: no
---

<h1 id="faq" class="page-header">{{ page.title }}</h1>

<p>This page discusses several questions that are often asked by our
users.</p>

<h3>Can I use Pinocchio for problems that have nothing to do with robotics?</h3>

<h3>When creating a Model from an urdf file, i cannot access any frames</h3>
- The frames are a way to keep track of special points (such as grippers etc..) in the kinematic chain. You can add a frame attached to a parent joint in the Model by hand but if you don't specify it yourself, no frames will be created automatically except when anchor/fixed joints are encountered in the urdf file. If it is the case, a frame is added at the center of the fixed joint and the next body inertial informations are merged to its parent
