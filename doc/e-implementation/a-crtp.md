# Curiously recurring template pattern
<!--
// Copyright (c) 2016 CNRS
// Author: Florent Lamiraux, Justin Carpentier, Florian Valenza
//
-->

Similarly to Eigen, Pinocchio library makes intensive use of the so called CRTP design
pattern. This pattern is used for performance reasons in the
implementation of static polymorphism, avoiding dynamic casts and calls to virtual methods.
All in all, CRTP plays a central role in the performance of Pinocchio.

We refer to  https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern for further explanations.
